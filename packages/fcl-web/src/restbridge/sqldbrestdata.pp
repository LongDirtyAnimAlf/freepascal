{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2019 by the Free Pascal development team

    SQLDB REST data manipulation routines.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit sqldbrestdata;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, sqldb, db, fpjson, sqldbrestio, sqldbrestschema;

Type
  TSQLQueryClass = Class of TSQLQuery;

  TRestFilterPair = Record
    Field : TSQLDBRestField;
    Operation : TRestFieldFilter;
    ValueParam : TParam;
    Value : String;
  end;
  TRestFilterPairArray = Array of TRestFilterPair;

  { TSQLDBRestDBHandler }

  TSQLDBRestDBHandler = Class(TComponent)
  private
    FDeriveResourceFromDataset: Boolean;
    FEmulateOffsetLimit: Boolean;
    FEnforceLimit: Int64;
    FExternalDataset: TDataset;
    FPostParams: TParams;
    FQueryClass: TSQLQueryClass;
    FRestIO: TRestIO;
    FStrings : TRestStringsConfig;
    FResource : TSQLDBRestResource;
    FOwnsResource : Boolean;
    procedure SetExternalDataset(AValue: TDataset);
  Protected
    function StreamRecord(O: TRestOutputStreamer; D: TDataset; FieldList: TRestFieldPairArray): Boolean; virtual;
    function FindExistingRecord(D: TDataset): Boolean;
    procedure CreateResourceFromDataset(D: TDataset); virtual;
    procedure DoNotFound; virtual;
    procedure SetPostParams(aParams: TParams; Old : TFields = Nil);virtual;
    procedure SetPostFields(aFields: TFields);virtual;
    procedure SetFieldFromData(DataField: TField; ResField: TSQLDBRestField; D: TJSONData); virtual;
    procedure InsertNewRecord; virtual;
    procedure UpdateExistingRecord(OldData: TDataset); virtual;
    Procedure Notification(AComponent: TComponent; Operation: TOperation); override;
    function SpecialResource: Boolean; virtual;
    function GetGeneratorValue(const aGeneratorName: String): Int64; virtual;
    function GetSpecialDatasetForResource(aFieldList: TRestFieldPairArray): TDataset; virtual;
    function FindFieldForParam(aOperation: TRestOperation; P: TParam): TSQLDBRestField; virtual;
    function BuildFieldList(ForceAll : Boolean): TRestFieldPairArray; virtual;
    function CreateQuery(aSQL: String): TSQLQuery; virtual;
    procedure FillParams(aOperation: TRestOperation; aParams: TParams;
      FilteredFields: TRestFilterPairArray); virtual;
    function GetDatasetForResource(aFieldList: TRestFieldPairArray; Singleton : Boolean): TDataset; virtual;
    function GetOrderByFieldArray: TRestFieldOrderPairArray;
    function GetOrderBy: UTF8String;virtual;
    function GetIDWhere(Out FilteredFields : TRestFilterPairArray): UTF8String; virtual;
    function GetWhere(Out FilteredFields : TRestFilterPairArray): UTF8String; virtual;
    function GetLimit: UTF8String;
    // Handle 4 basic operations
    procedure DoHandleGet;virtual;
    procedure DoHandleDelete;virtual;
    procedure DoHandlePost;virtual;
    procedure DoHandlePut; virtual;
    // Parameters used when executing update SQLs. Used to get values for return dataset params.
    Property PostParams : TParams Read FPostParams;
  Public
    Destructor Destroy; override;
    // Get limi
    Function GetLimitOffset(out aLimit, aOffset: Int64) : Boolean; virtual;
    Procedure Init(aIO: TRestIO; aStrings : TRestStringsConfig;AQueryClass : TSQLQueryClass); virtual;
    Procedure ExecuteOperation;
    Function StreamDataset(O: TRestOutputStreamer; D: TDataset; FieldList: TRestFieldPairArray; CurrentOnly : Boolean = False) : Int64;
    procedure SetParamFromData(P: TParam; F: TSQLDBRestField; D: TJSONData); virtual;
    function GetDataForParam(P: TParam; F: TSQLDBRestField; Sources : TVariableSources = AllVariableSources): TJSONData; virtual;
    Function GetString(aString : TRestStringProperty) : UTF8String;
    Property IO : TRestIO Read FRestIO;
    Property Strings : TRestStringsConfig Read FStrings;
    Property QueryClass : TSQLQueryClass Read FQueryClass;
    Property EnforceLimit : Int64 Read FEnforceLimit Write FEnforceLimit;
    Property ExternalDataset : TDataset Read FExternalDataset Write SetExternalDataset;
    Property EmulateOffsetLimit : Boolean Read FEmulateOffsetLimit Write FEmulateOffsetLimit;
    Property DeriveResourceFromDataset : Boolean Read FDeriveResourceFromDataset Write FDeriveResourceFromDataset;
  end;
  TSQLDBRestDBHandlerClass = class of TSQLDBRestDBHandler;


implementation

uses strutils, variants, dateutils, base64, sqldbrestconst;


Const
  FilterParamPrefix : Array [TRestFieldFilter] of string = ('eq_','lt_','gt_','lte_','gte_','');
  FilterOps : Array [TRestFieldFilter] of string = ('=','<','>','<=','>=','IS NULL');

{ TSQLDBRestDBHandler }


procedure TSQLDBRestDBHandler.Init(aIO: TRestIO; aStrings: TRestStringsConfig; AQueryClass: TSQLQueryClass);
begin
  FRestIO:=aIO;
  FQueryClass:=aQueryClass;
  FStrings:=aStrings;
end;

procedure TSQLDBRestDBHandler.ExecuteOperation;

begin
  if Not DeriveResourceFromDataset then
    FResource:=IO.Resource;
  Case IO.Operation of
    roGet : DoHandleGet;
    roPut : DoHandlePut;
    roPost : DoHandlePost;
    roDelete : DoHandleDelete;
  end;
end;

function TSQLDBRestDBHandler.GetString(aString: TRestStringProperty): UTF8String;
begin
  if Assigned(FStrings) then
    Result:=FStrings.GetRestString(aString)
  else
    Result:=TRestStringsConfig.GetDefaultString(aString);
end;


function TSQLDBRestDBHandler.GetIDWhere(out FilteredFields: TRestFilterPairArray): UTF8String;

Var
  Qry : UTF8String;
  L : TSQLDBRestFieldArray;
  F: TSQLDBRestField;
  I : Integer;

begin
  FilteredFields:=Default(TRestFilterPairArray);
  Result:='';
  if (IO.GetVariable('ID',Qry,[vsQuery,vsRoute,vsHeader])=vsNone) or (Qry='') then
    if not Assigned(PostParams) then
      raise ESQLDBRest.Create(IO.RestStatuses.GetStatusCode(rsInvalidParam),SErrNoKeyParam);
  L:=FResource.GetFieldArray(flWhereKey);
  SetLength(FilteredFields,Length(L));
  for I:=0 to Length(L)-1 do
    begin
    F:=L[i];
    FilteredFields[I].Field:=F;
    FilteredFields[I].Operation:=rfEqual;
    // If we have postparams, it means we're building a dataset for return data.
    // So check for actual DB value there
    if Assigned(PostParams) then
      FilteredFields[I].ValueParam:=PostParams.FindParam(F.FieldName);
    if (FilteredFields[I].ValueParam=nil) then
      FilteredFields[I].Value:=ExtractWord(1,Qry,['|']);
    If (Result<>'') then
      Result:=Result+' and ';
    Result:='('+F.FieldName+' = :'+FilterParamPrefix[rfEqual]+F.FieldName+')';
    end;
end;

function TSQLDBRestDBHandler.GetWhere(out FilteredFields: TRestFilterPairArray
  ): UTF8String;

Const
  MaxFilterCount = 1+ Ord(High(TRestFieldFilter)) - Ord(Low(TRestFieldFilter));

Var
  Qry : UTF8String;
  L : TSQLDBRestFieldArray;
  RF : TSQLDBRestField;
  fo : TRestFieldFilter;
  aLen : integer;

begin
  FilteredFields:=Default(TRestFilterPairArray);
  Result:='';
  L:=FResource.GetFieldArray(flFilter);
  SetLength(FilteredFields,Length(L)*MaxFilterCount);
  aLen:=0;
  for RF in L do
    for FO in RF.Filters do
      if IO.GetFilterVariable(RF.PublicName,FO,Qry)<>vsNone then
        begin
        FilteredFields[aLen].Field:=RF;
        FilteredFields[aLen].Operation:=FO;
        FilteredFields[aLen].Value:=Qry;
        Inc(aLen);
        If (Result<>'') then Result:=Result+' AND ';
        if FO<>rfNull then
          Result:=Result+Format('(%s %s :%s%s)',[RF.FieldName,FilterOps[FO],FilterParamPrefix[FO],RF.FieldName])
        else
          Case IO.StrToNullBoolean(Qry,True) of
            nbTrue : Result:=Result+Format('(%s IS NULL)',[RF.FieldName]);
            nbFalse : Result:=Result+Format('(%s IS NOT NULL)',[RF.FieldName]);
            nbNone :  Raise ESQLDBRest.CreateFmt(IO.RestStatuses.GetStatusCode(rsInvalidParam),SErrInvalidBooleanForField,[RF.PublicName])
          end;
        end;
  SetLength(FilteredFields,aLen);
end;

function TSQLDBRestDBHandler.GetOrderByFieldArray : TRestFieldOrderPairArray;

  Procedure AddField(Idx : Integer; F : TSQLDBRestField; aDesc : boolean);

  begin
    Result[Idx].RestField:=F;
    Result[Idx].Desc:=aDesc;
  end;

Var
  L : TSQLDBRestFieldArray;
  I,J,aLen : Integer;
  F : TSQLDBRestField;
  V,FN : UTF8String;
  Desc : Boolean;

begin
  Result:=Default(TRestFieldOrderPairArray);
  if IO.GetVariable(GetString(rpOrderBy),V,[vsQuery])=vsNone then
    begin
    L:=FResource.GetFieldArray(flWhereKey);
    SetLength(Result,Length(L));
    I:=0;
    For F in L do
      begin
      AddField(I,F,False);
      Inc(I);
      end
    end
  else
    begin
    L:=FResource.GetFieldArray(flOrderBy);
    aLen:=WordCount(V,[',']);
    SetLength(Result,aLen);
    For I:=1 to WordCount(V,[',']) do
      begin
      FN:=ExtractWord(I,V,[',']);
      Desc:=SameText(ExtractWord(2,FN,[' ']),'desc');
      FN:=ExtractWord(1,FN,[' ']);
      J:=Length(L)-1;
      While (J>=0) and Not SameText(L[J].PublicName,FN) do
        Dec(J);
      if J<0 then
        Raise ESQLDBRest.CreateFmt(IO.RestStatuses.GetStatusCode(rsInvalidParam),SErrInvalidSortField,[FN]);
      F:=L[J];
      if Desc then
        if not (foOrderByDesc in F.Options) then
          Raise ESQLDBRest.CreateFmt(IO.RestStatuses.GetStatusCode(rsInvalidParam),SErrInvalidSortDescField,[FN]);
      AddField(I-1,F,Desc)
      end;
    end;
end;

function TSQLDBRestDBHandler.GetOrderBy: UTF8String;

Const
  AscDesc : Array[Boolean] of string = ('ASC','DESC');

Var
  L : TRestFieldOrderPairArray;
  P : TRestFieldOrderPair;

begin
  Result:='';
  L:=GetOrderByFieldArray;
  For P in L do
    begin
    if Result<>'' then
      Result:=Result+', ';
    Result:=Result+P.RestField.FieldName+' '+AscDesc[P.Desc];
    end;
end;

function TSQLDBRestDBHandler.CreateQuery(aSQL: String): TSQLQuery;

begin
  Result:=FQueryClass.Create(Self);
  Result.DataBase:=IO.Connection;
  Result.Transaction:=IO.Transaction;
  Result.SQL.Text:=aSQL;
end;

function TSQLDBRestDBHandler.BuildFieldList(ForceAll : Boolean): TRestFieldPairArray;

Var
  L : TSQLDBRestFieldArray;
  F : TSQLDBRestField;
  aCount : Integer;
  Fi,Fe : TStrings;

  Function ML(N : String) : TStrings;
  Var
    V : UTF8String;
  begin
    Result:=Nil;
    if ForceAll then
      exit;
    IO.GetVariable(N,V);
    if (V<>'') then
      begin
      Result:=TStringList.Create;
      Result.StrictDelimiter:=True;
      Result.CommaText:=V;
      end;
  end;

  Function IsIncluded(F : TSQLDBRestField) : Boolean;
  begin
    Result:=(FI=Nil) or (FI.IndexOf(F.PublicName)<>-1)
  end;

  Function IsExcluded(F : TSQLDBRestField) : Boolean;
  begin
    Result:=(FE<>Nil) and (FE.IndexOf(F.PublicName)<>-1)
  end;

begin
  Result:=Default(TRestFieldPairArray);
  if Not Assigned(FResource) then
    exit;
  FE:=Nil;
  FI:=ML(GetString(rpFieldList));
  try
    FE:=ML(GetString(rpExcludeFieldList));
    L:=FResource.GetFieldArray(flSelect);
    SetLength(Result,Length(L));
    aCount:=0;
    For F in L do
      if IsIncluded(F) and not IsExcluded(F) then
        begin
        Result[aCount].RestField:=F;
        Result[aCount].DBField:=Nil;
        Inc(aCount);
        end;
     SetLength(Result,aCount);
  finally
    FI.Free;
    FE.Free;
  end;
end;

function TSQLDBRestDBHandler.GetDataForParam(P: TParam; F: TSQLDBRestField;
  Sources: TVariableSources): TJSONData;

Var
  vs : TVariableSource;
  S,N : UTF8String;

begin
  Result:=Nil;
  if Assigned(F) then
    begin
    N:=F.PublicName;
    vs:=IO.GetVariable(N,S,Sources);
    if (vs<>vsNone) then
      Result:=TJSONString.Create(S)
    else if (vsContent in Sources) then
      Result:=IO.RESTInput.GetContentField(N);
    end;
  If (Result=Nil) then
    begin
    N:=P.Name;
    if N='ID_' then
      N:='ID';
    vs:=IO.GetVariable(N,S);
    if (vs<>vsNone) then
      Result:=TJSONString.Create(S)
    else if (vsContent in Sources) then
      Result:=IO.RESTInput.GetContentField(N)
    end;
end;

procedure TSQLDBRestDBHandler.SetParamFromData(P: TParam; F: TSQLDBRestField;
  D: TJSONData);

Var
  S : String;

begin
  if Assigned(D) then
    S:=D.AsString;
  if not Assigned(D) then
    P.Clear
  else if Assigned(F) then
    Case F.FieldType of
      rftInteger : P.AsInteger:=D.AsInteger;
      rftLargeInt : P.AsLargeInt:=D.AsInt64;
      rftFloat : P.AsFloat:=D.AsFloat;
      rftDate : P.AsDateTime:=ScanDateTime(GetString(rpDateFormat),D.AsString);
      rftTime : P.AsDateTime:=ScanDateTime(GetString(rpTimeFormat),D.AsString);
      rftDateTime : P.AsDateTime:=ScanDateTime(GetString(rpDateTimeFormat),D.AsString);
      rftString : P.AsString:=D.AsString;
      rftBoolean : P.AsBoolean:=D.AsBoolean;
      rftBlob :
{$IFNDEF VER3_0}
         P.AsBlob:=BytesOf(DecodeStringBase64(D.AsString));
{$ELSE}
         P.AsBlob:=DecodeStringBase64(D.AsString);
{$ENDIF}
    else
      P.AsString:=D.AsString;
    end
  else
    P.AsString:=D.AsString;
end;

function TSQLDBRestDBHandler.FindFieldForParam(aOperation: TRestOperation;
  P: TParam): TSQLDBRestField;

Var
  N : UTF8String;
  A : TSQLDBRestFieldArray;

begin
  Result:=Nil;
  N:=P.Name;
  if (N='ID_') then
    begin
    A:=FResource.GetFieldArray(flWhereKey);
    if (Length(A)=1) then
      Result:=A[0];
    end
  else
    Result:=FResource.Fields.FindByFieldName(N);
end;

procedure TSQLDBRestDBHandler.FillParams(aOperation : TRestOperation; aParams: TParams;FilteredFields : TRestFilterPairArray);

Var
  I : Integer;
  P : TParam;
  D : TJSONData;
  F : TSQLDBRestField;
  FF : TRestFilterPair;
  Sources : TVariableSources;


begin
  // Fill known params
  for FF in FilteredFields do
    begin
    F:=FF.Field;
    if FF.Operation<>rfNull then
      begin
      P:=aParams.FindParam(FilterParamPrefix[FF.Operation]+F.FieldName);
      // If there is no %where% macro, the parameter can be absent
      if Assigned(P) then
        begin
        if Assigned(FF.ValueParam) then
          P.Value:=FF.ValueParam.Value
        else
          begin
          D:=TJSONString.Create(FF.Value);
          try
            SetParamFromData(P,F,D)
          finally
            D.Free;
          end;
          end;
        end;
      end;
    end;
  // Fill in remaining params. Determine source
  case aOperation of
    roGet : Sources:=[vsQuery,vsRoute];
    roPost,
    roPut : Sources:=[vsQuery,vsContent,vsRoute];
    roDelete : Sources:=[vsQuery,vsRoute];
  else
    Sources:=AllVariableSources;
  end;
  For I:=0 to aParams.Count-1 do
    begin
    P:=aParams[i];
    if P.IsNull then
      try
        D:=Nil;
        F:=FindFieldForParam(aOperation,P);
        D:=GetDataForParam(P,F,Sources);
        if (D<>Nil) then
          SetParamFromData(P,F,D)
        else if (aOperation in [roDelete]) then
          Raise ESQLDBRest.CreateFmt(IO.RestStatuses.GetStatusCode(rsInvalidParam),SErrMissingParameter,[P.Name])
        else
          P.Clear;
      finally
        FreeAndNil(D);
      end;
    end;
end;

function TSQLDBRestDBHandler.GetLimitOffset(out aLimit, aOffset: Int64
  ): Boolean;

begin
  Result:=IO.GetLimitOffset(EnforceLimit,aLimit,aoffset);
end;

function TSQLDBRestDBHandler.GetLimit: UTF8String;

var
  aOffset, aLimit : Int64;
  CT : String;

begin
  Result:='';
  GetLimitOffset(aLimit,aOffset);
  if aLimit=0 then
    exit;
  if Not (IO.Connection is TSQLConnector) then
    Raise ESQLDBRest.Create(IO.RestStatuses.GetStatusCode(rsError),SErrLimitNotSupported);
  CT:=lowerCase(TSQLConnector(IO.Connection).ConnectorType);
  if Copy(CT,1,5)='mysql' then
    CT:='mysql';
  case CT of
    'mysql' : Result:=Format('LIMIT %d, %d',[aOffset,aLimit]);
    'postgresql',
    'sqlite3' : Result:=Format('LIMIT %d offset %d',[aLimit,aOffset]);
    'interbase',
    'firebird' : Result:=Format('ROWS %d TO %d',[aOffset,aOffset+aLimit-1]);
    'oracle',
    'sybase',
    'odbc',
    'MSSQLServer' : Result:=Format('OFFSET %d ROWS FETCH NEXT %d ROWS ONLY',[aOffset,aLimit]);
  end;
end;


function TSQLDBRestDBHandler.StreamRecord(O: TRestOutputStreamer; D: TDataset;
  FieldList: TRestFieldPairArray): Boolean;

Var
  i : Integer;

begin
  Result:=IO.Resource.AllowRecord(IO.RestContext,D);
  if not Result then
    exit;
  O.StartRow;
  For I:=0 to Length(FieldList)-1 do
    O.WriteField(FieldList[i]);
  O.EndRow;
end;

function TSQLDBRestDBHandler.StreamDataset(O: TRestOutputStreamer; D: TDataset;
  FieldList: TRestFieldPairArray; CurrentOnly : Boolean = False): Int64;

Var
  aLimit,aOffset : Int64;

  Function LimitReached : boolean;

  begin
    Result:=EmulateOffsetLimit and (aLimit<=0);
  end;

Var
  I : Integer;

begin
  Result:=0;
  if EmulateOffsetLimit then
    GetLimitOffset(aLimit,aOffset)
  else
    begin
    aLimit:=0;
    aOffset:=0;
    end;
  For I:=0 to Length(FieldList)-1 do
    FieldList[i].DBField:=D.FieldByName(FieldList[i].RestField.FieldName);
  if O.HasOption(ooMetadata) then
    O.WriteMetadata(FieldList);
  O.StartData;
  if CurrentOnly then
    StreamRecord(O,D,FieldList)
  else
    begin
    if EmulateOffsetLimit then
      While (aOffset>0) and not D.EOF do
        begin
        D.Next;
        Dec(aOffset);
        end;
    While not (D.EOF or LimitReached) do
      begin
      If StreamRecord(O,D,FieldList) then
        begin
        Dec(aLimit);
        inc(Result);
        end;
      D.Next;
      end;
    end;
  O.EndData;
end;

function TSQLDBRestDBHandler.GetSpecialDatasetForResource(
  aFieldList: TRestFieldPairArray): TDataset;


Var
  aLimit,aOffset : Int64;

begin
  Result:=ExternalDataset;
  if (Result=Nil) then
    begin
    GetLimitOffset(aLimit,aOffset);
    Result:=FResource.GetDataset(IO.RestContext,aFieldList,GetOrderByFieldArray,aLimit,aOffset);
    end;
end;

procedure TSQLDBRestDBHandler.SetExternalDataset(AValue: TDataset);
begin
  if FExternalDataset=AValue then Exit;
  if Assigned(FExternalDataset) then
    FExternalDataset.RemoveFreeNotification(Self);
  FExternalDataset:=AValue;
  if Assigned(FExternalDataset) then
    FExternalDataset.FreeNotification(Self);
end;

function TSQLDBRestDBHandler.SpecialResource: Boolean;

begin
  Result:=(ExternalDataset<>Nil) or Assigned(FResource.OnGetDataset);
end;

function TSQLDBRestDBHandler.GetDatasetForResource(aFieldList: TRestFieldPairArray; Singleton : Boolean): TDataset;

Var
  aWhere,aOrderby,aLimit,SQL : UTF8String;
  Q : TSQLQuery;
  WhereFilterList : TRestFilterPairArray;

begin
  if SpecialResource then
    Exit(GetSpecialDatasetForResource(aFieldList));
  if Singleton then
    aWhere:=GetIDWhere(WhereFilterList)
  else
    aWhere:=GetWhere(WhereFilterList);
  aOrderBy:=GetOrderBy;
  aLimit:=GetLimit;
  SQL:=FResource.GetResolvedSQl(skSelect,aWhere,aOrderBy,aLimit);
  Q:=CreateQuery(SQL);
  Try
    Q.UsePrimaryKeyAsKey:=False;
    FillParams(roGet,Q.Params,WhereFilterList);
    Result:=Q;
  except
    Q.Free;
    raise;
  end;
end;

procedure TSQLDBRestDBHandler.CreateResourceFromDataset(D : TDataset);

begin
  FOwnsResource:=True;
  FResource:=TCustomViewResource.Create(Nil);
  FResource.PopulateFieldsFromFieldDefs(D.FieldDefs,Nil,Nil,[]);
end;

procedure TSQLDBRestDBHandler.DoNotFound;

begin
  IO.Response.Code:=IO.RestStatuses.GetStatusCode(rsRecordNotFound);
  IO.Response.CodeText:='NOT FOUND';  // Do not localize
  IO.CreateErrorResponse;
end;

procedure TSQLDBRestDBHandler.DoHandleGet;

Var
  D : TDataset;
  FieldList : TRestFieldPairArray;
  qID : UTF8string;
  Single : Boolean;

begin
  FieldList:=BuildFieldList(False);
  Single:=(IO.GetVariable('ID',qId,[vsRoute,vsQuery])<>vsNone);
  D:=GetDatasetForResource(FieldList,Single);
  try
    D.Open;
    if DeriveResourceFromDataset then
      begin
      CreateResourceFromDataset(D);
      FieldList:=BuildFieldList(False);
      end;
    if not (D.EOF and D.BOF) then
      StreamDataset(IO.RESTOutput,D,FieldList)
    else
      begin
      if Single then
        DoNotFound
      else
        StreamDataset(IO.RESTOutput,D,FieldList)
      end;
  finally
    D.Free;
  end;
end;

function TSQLDBRestDBHandler.GetGeneratorValue(const aGeneratorName: String
  ): Int64;

begin
{$IFDEF VER3_0_4}
  // The 'get next value' SQL in 3.0.4 is wrong, so we need to do this sep
  if (IO.Connection is TSQLConnector) and SameText((IO.Connection as TSQLConnector).ConnectorType,'Sqlite3') then
    begin
    With CreateQuery('SELECT seq+1 FROM sqlite_sequence WHERE name=:aName') do
      Try
        ParamByName('aName').AsString:=aGeneratorName;
        Open;
        if (EOF and BOF) then
          DatabaseErrorFmt('Generator %s does not exist',[aGeneratorName]);
        Result:=Fields[0].asLargeint;
      Finally
        Free;
      end;
    end
  else
{$ENDIF}
  Result:=IO.Connection.GetNextValue(aGeneratorName,1);
end;

procedure TSQLDBRestDBHandler.SetPostFields(aFields : TFields);

Var
  I : Integer;
  FData : TField;
  D : TJSONData;
  RF : TSQLDBRestField;
  V : UTF8string;

begin
  // Another approach would be to create params for all fields,
  // call setPostParams, and copy field data from all set params
  // That would allow the use of checkparams...
  For I:=0 to aFields.Count-1 do
    try
      D:=Nil;
      FData:=aFields[i];
      RF:=FResource.Fields.FindByFieldName(FData.FieldName);
      if (RF<>Nil) then
        begin
        if (RF.GeneratorName<>'')  then // Only when doing POST
          D:=TJSONInt64Number.Create(GetGeneratorValue(RF.GeneratorName))
        else
          D:=IO.RESTInput.GetContentField(RF.PublicName);
        end
      else if IO.GetVariable(FData.Name,V,[vsContent,vsQuery])<>vsNone then
        D:=TJSONString.Create(V);
      if (D<>Nil) then
        SetFieldFromData(FData,RF,D); // Use new value, if any
    finally
      D.Free;
    end;
end;

procedure TSQLDBRestDBHandler.SetFieldFromData(DataField: TField; ResField: TSQLDBRestField; D: TJSONData);

begin
  if not Assigned(D) then
    DataField.Clear
  else if Assigned(ResField) then
    Case ResField.FieldType of
      rftInteger : DataField.AsInteger:=D.AsInteger;
      rftLargeInt : DataField.AsLargeInt:=D.AsInt64;
      rftFloat : DataField.AsFloat:=D.AsFloat;
      rftDate : DataField.AsDateTime:=ScanDateTime(GetString(rpDateFormat),D.AsString);
      rftTime : DataField.AsDateTime:=ScanDateTime(GetString(rpTimeFormat),D.AsString);
      rftDateTime : DataField.AsDateTime:=ScanDateTime(GetString(rpDateTimeFormat),D.AsString);
      rftString : DataField.AsString:=D.AsString;
      rftBoolean : DataField.AsBoolean:=D.AsBoolean;
      rftBlob :
{$IFNDEF VER3_0}
         DataField.AsBytes:=BytesOf(DecodeStringBase64(D.AsString));
{$ELSE}
         DataField.AsString:=DecodeStringBase64(D.AsString);
{$ENDIF}
    else
      DataField.AsString:=D.AsString;
    end
  else
    DataField.AsString:=D.AsString;
end;


procedure TSQLDBRestDBHandler.SetPostParams(aParams : TParams; Old : TFields = Nil);

Var
  I : Integer;
  P : TParam;
  D : TJSONData;
  F : TSQLDBRestField;
  FOld : TField;
  V : UTF8string;

begin
  For I:=0 to aParams.Count-1 do
    try
      D:=Nil;
      FOld:=Nil;
      P:=aParams[i];
      F:=FResource.Fields.FindByFieldName(P.Name);
      If Assigned(Old) then
        Fold:=Old.FindField(P.Name);
      if (F<>Nil) then
        begin
        if (F.GeneratorName<>'') and (Old=Nil) then // Only when doing POST
          D:=TJSONInt64Number.Create(GetGeneratorValue(F.GeneratorName))
        else
          D:=IO.RESTInput.GetContentField(F.PublicName);
        end
      else if IO.GetVariable(P.Name,V,[vsContent,vsQuery])<>vsNone then
        D:=TJSONString.Create(V);
      if (D=Nil) and Assigned(Fold) then
        P.AssignFromField(Fold) // use old value
      else
        SetParamFromData(P,F,D); // Use new value, if any
    finally
      D.Free;
    end;
  // Give user a chance to look at it.
  FResource.CheckParams(io.RestContext,roPost,aParams);
  // Save so it can be used in GetWHereID for return
  FPostParams:=TParams.Create(TParam);
  FPostParams.Assign(aParams);
end;

procedure TSQLDBRestDBHandler.InsertNewRecord;

Var
  S : TSQLStatement;
  SQL : UTF8String;

begin
  if Assigned(ExternalDataset) then
    begin
    ExternalDataset.Append;
    SetPostFields(ExternalDataset.Fields);
    try
      ExternalDataset.Post;
    except
      ExternalDataset.Cancel;
      Raise;
    end
    end
  else
    begin
    SQL:=FResource.GetResolvedSQl(skInsert,'','','');
    S:=TSQLStatement.Create(Self);
    try
      S.Database:=IO.Connection;
      S.Transaction:=IO.Transaction;
      S.SQL.Text:=SQL;
      SetPostParams(S.Params);
      S.Execute;
      PostParams.Assign(S.Params);
      S.Transaction.Commit;
    Finally
      S.Free;
    end;
    end;
end;

procedure TSQLDBRestDBHandler.DoHandlePost;

Var
  D : TDataset;
  FieldList : TRestFieldPairArray;

begin
  // We do this first, so we don't run any unnecessary queries
  if not IO.RESTInput.SelectObject(0) then
    raise ESQLDBRest.Create(IO.RestStatuses.GetStatusCode(rsInvalidParam), SErrNoResourceDataFound);
  InsertNewRecord;
  // Now build response. We can imagine not doing a select again, and simply supply back the fields as sent...
  FieldList:=BuildFieldList(False);
  D:=GetDatasetForResource(FieldList,True);
  try
    D.Open;
    IO.RESTOutput.OutputOptions:=IO.RESTOutput.OutputOptions-[ooMetadata];
    StreamDataset(IO.RESTOutput,D,FieldList);
  finally
    D.Free;
  end;
end;

procedure TSQLDBRestDBHandler.UpdateExistingRecord(OldData : TDataset);

Var
  S : TSQLStatement;
  SQl : String;
  WhereFilterList : TRestFilterPairArray;

begin
  if (OldData=ExternalDataset) then
    begin
    ExternalDataset.Edit;
    try
      SetPostFields(ExternalDataset.Fields);
      ExternalDataset.Post;
    except
      ExternalDataset.Cancel;
      Raise;
    end
    end
  else
    begin
    SQL:=FResource.GetResolvedSQl(skUpdate,GetIDWhere(WhereFilterList) ,'','');
    S:=TSQLStatement.Create(Self);
    try
      S.Database:=IO.Connection;
      S.Transaction:=IO.Transaction;
      S.SQL.Text:=SQL;
      SetPostParams(S.Params,OldData.Fields);
      FillParams(roGet,S.Params,WhereFilterList);
      // Give user a chance to look at it.
      FResource.CheckParams(io.RestContext,roPut,S.Params);
      S.Execute;
      S.Transaction.Commit;
    finally
      S.Free;
    end;
    end;
end;

Function TSQLDBRestDBHandler.FindExistingRecord(D : TDataset) : Boolean;

Var
  KeyFields : String;
  FieldList : TRestFilterPairArray;
  FP : TRestFilterPair;
  V : Variant;
  I : Integer;

begin
  D.Open;
  if D<>ExternalDataset then
    Result:=Not (D.BOF and D.EOF)
  else
    begin
    GetIDWhere(FieldList);
    V:=VarArrayCreate([0,Length(FieldList)-1],varVariant);
    KeyFields:='';
    I:=0;
    For FP in FieldList do
      begin
      if KeyFields<>'' then
        KeyFields:=KeyFields+';';
      KeyFields:=KeyFields+FP.Field.FieldName;
      if Assigned(FP.ValueParam) then
        V[i]:=FP.ValueParam.Value
      else
        V[i]:=FP.Value;
      Inc(i);
      end;
    Result:=D.Locate(KeyFields,V,[loCaseInsensitive]);
    end;
end;

procedure TSQLDBRestDBHandler.DoHandlePut;

Var
  D : TDataset;
  FieldList : TRestFieldPairArray;

begin
  // We do this first, so we don't run any unnecessary queries
  if not IO.RESTInput.SelectObject(0) then
    Raise ESQLDBRest.Create(IO.RestStatuses.GetStatusCode(rsInvalidParam),SErrNoResourceDataFound);
  // Get the original record.
  FieldList:=BuildFieldList(True);
  D:=GetDatasetForResource(FieldList,True);
  try
    if not FindExistingRecord(D) then
      begin
      DoNotFound;
      exit;
      end;
    UpdateExistingRecord(D);
    // Now build response
    if D<>ExternalDataset then
      begin;
      // Now build response. We can imagine not doing a select again, and simply supply back the fields as sent...
      FreeAndNil(D);
      D:=GetDatasetForResource(FieldList,True);
      FieldList:=BuildFieldList(False);
      D.Open;
      end;
    IO.RESTOutput.OutputOptions:=IO.RESTOutput.OutputOptions-[ooMetadata];
    StreamDataset(IO.RESTOutput,D,FieldList);
  finally
    D.Free;
  end;
end;

destructor TSQLDBRestDBHandler.Destroy;
begin
  FreeAndNil(FPostParams);
  If FOwnsResource then
     FreeAndNil(FResource);
  inherited Destroy;
end;

procedure TSQLDBRestDBHandler.Notification(AComponent: TComponent; Operation: TOperation);
begin
  If Operation=opRemove then
    begin
    if (aComponent=FExternalDataset) then
      FExternalDataset:=Nil;
    end;
end;

procedure TSQLDBRestDBHandler.DoHandleDelete;

Var
  aWhere,SQL : UTF8String;
  Q : TSQLQuery;
  FilteredFields : TRestFilterPairArray;

begin
  if Assigned(ExternalDataset) then
    begin
    If FindExistingRecord(ExternalDataset) then
      ExternalDataset.Delete
    else
      DoNotFound;
    end
  else
    begin
    aWhere:=GetIDWhere(FilteredFields);
    SQL:=FResource.GetResolvedSQl(skDelete,aWhere,'');
    Q:=CreateQuery(SQL);
    try
      FillParams(roDelete,Q.Params,FilteredFields);
      Q.ExecSQL;
      if Q.RowsAffected<>1 then
        DoNotFound;
    finally
      Q.Free;
    end;
    end;
end;

end.


{ %fail }
{ %opt=-Sew }
{$mode iso}

var
  l: longint;
begin
  l:=1;
  case l of
    2: writeln;
  end;
end.

function [ Para_integer ] = Math_Integer( Para_last,Para_dot_last,Para_dot,DT )
    Para_integer = Para_last + 0.5*(Para_dot + Para_dot_last)*DT; 
end


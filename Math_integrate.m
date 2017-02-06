function [ Para_integrate ] = Math_integrate( Para_last,Para_dot_last,Para_dot,DT )
    Para_integrate = Para_last + 0.5*(3 * Para_dot - Para_dot_last)*DT; 
end


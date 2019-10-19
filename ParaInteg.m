function Para = ParaInteg( Para_last,Para_dot_last,Para_dot,DeltaT )

    Para = Para_last + 0.5 * DeltaT *(Para_dot_last + Para_dot);

end


function Para = ControlLimit( In,In_last,Bound,Rate,DeltaT )

    if((In - In_last) > Rate * DeltaT)
        Para = In_last + Rate * DeltaT;
    elseif((In - In_last) < -Rate * DeltaT)
        Para = In_last - Rate * DeltaT;
    else
        Para = In;
    end

    if(Para > Bound)
        Para = Bound;
    elseif(Para < -Bound)
        Para = -Bound;
    else
        Para = Para;
    end

end


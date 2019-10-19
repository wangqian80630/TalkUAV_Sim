function Para = ParaLimit( In,DownBound,UpBound )
    
    if(In >= UpBound)
        Para = UpBound;
    elseif(In <= DownBound)
        Para = DownBound;
    else
        Para = In;
    end

end


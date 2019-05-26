function inverted=invertHT(inputHT)
%Inverting a Homogenous Transformation Matrix

    if size(inputHT,1) ~= size(inputHT,2)
        error('R must be square');
    end

    if size(inputHT,1) ==3
       R=inputHT(1:2,1:2);
       t=inputHT(1:2,3);
    end
    
    if size(inputHT,1) ==4
        R=inputHT(1:3,1:3);
        t=inputHT(1:3,4);
    end

        invR=R';
        invt=-R'*t;
        inverted = [invR invt; zeros(1,size(invR,2)) 1];



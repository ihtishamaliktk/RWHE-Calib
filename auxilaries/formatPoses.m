function [methodsNames,Hhand2eye,Hbase2grid,timeout]=formatPoses(handEyeHT,base2grid,time);
% changes the format of the poses in order to be used wiht computeErrors
% function

Hbase2grid=[];
methodsNames = fieldnames(handEyeHT);
for t=1:size(methodsNames,1)
    name=methodsNames{t};
    Hhand2eye(:,:,t)= getfield(handEyeHT,name);
    timeout(t,1)= getfield(time,name);

        if isfield(base2grid,name)
            Hbase2grid(:,:,t)= getfield(base2grid,name);
        else
            Hbase2grid(:,:,t)=nan(4);
        end
end

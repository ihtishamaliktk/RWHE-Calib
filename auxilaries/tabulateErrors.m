function tabulateErrors(methodsNames,form,errs,time)

%the function takes all the homogenous transformations converts them to
%eular angles and prints in a tabular form for easy understanding

%% Print Results
%re-arranging data

Err.form=form;         
Err.Rerr=errs(:,1); Err.terr=errs(:,2); Err.reproj=errs(:,3);
Err.Rabs=errs(:,4); Err.tabs=errs(:,5);
     

ResultsTT = table(Err.form,time,Err.Rerr,(Err.terr)*1000,Err.reproj,Err.Rabs,(Err.tabs)*1000);
ResultsTT.Properties.VariableNames = {'EvalForm','Time_sec','Rerr_deg','terr_mm','reproj_px','RAbs_deg','tabs_mm'};
ResultsTT.Properties.RowNames  = methodsNames;

ResultsTT

end

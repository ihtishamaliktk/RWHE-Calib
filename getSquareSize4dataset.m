function squareSize=getSquareSize4dataset(DatasetName)
 
% get the square size for the checker boards for the mentioned datasets,For
% custom dataset  specify size manually


switch DatasetName
        
    case {'CS_synthetic_1','CS_synthetic_2','CS_synthetic_3'}
        squareSize=0.2; % meters  or 20cm
        
    case {'kuka_1'}
        squareSize=0.02; % meters  or 20mm
    case {'kuka_2'}
        squareSize=0.015 % meters  or 15mm
    case {'kuka_3'}
       squareSize=0.06 % meters  or 60mm
               
    otherwise
        warning('Custom dataset selected')
        squareSize= str2num(input('Please input the correct square size in m: ','s'));

end
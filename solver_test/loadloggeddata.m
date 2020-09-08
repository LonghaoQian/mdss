% read the datalog file and store it into matlab .mat file
function loadloggeddata(filename)
fileID = fopen([filename '.txt'],'r');
%% read the first line to determine the number of data
disp(['Processing log file with name: ' filename '.txt ...'])
identifier = fgetl(fileID);
if(strcmp(identifier,'This file is a data log from solver.'))
    disp('This is a data log file...')
    %% read the second line to get number of data columns
    logged_data.num_of_columns = str2double(fgetl(fileID));
    disp(['The file contains ' num2str(logged_data.num_of_columns) ' columns.'])
    %% store the tags into a map
    tagkey = cell(0,1); 
    disp('The following tags are found: ')
    for i = 1 : logged_data.num_of_columns + 1 % t time will always be the first tag
        tagkey{end+1,1} = fgetl(fileID);
        disp([tagkey{i,1} ','])
    end
    disp('End of tag list...')
    valueset = 1:1:logged_data.num_of_columns + 1;
    logged_data.taglist = tagkey;
    logged_data.tagmap = containers.Map(tagkey,valueset);% store the map in the container
    %% store all data into a cell
    logged_data.data = zeros(1,logged_data.num_of_columns + 1);
    data_pointer = 1;
    tline = fgetl(fileID);
    while ischar(tline)
        [~,NumIndex] = regexp(tline,'*','match','split');
        for i = 1:logged_data.num_of_columns + 1
            logged_data.data(data_pointer,i) = str2double(NumIndex{i});
        end
         tline = fgetl(fileID);
         data_pointer  = data_pointer + 1;
    end
    [logged_data.num_of_frames,~] = size(logged_data.data);
    disp(['Scanning finished, ' num2str(logged_data.num_of_frames) ' frames are found. ' ])
    save([filename '.mat'],'logged_data')
else
    disp('This is not a data log file, exiting...')
end
fclose(fileID);
disp('Done')
% plot the selected data based on datalog
function drawlogfromtag(filename,tagname)
load(filename,'logged_data');
if exist('logged_data','var')==1
    % determine whether the tag name exists
    if any(strcmp(logged_data.taglist,tagname))
        figure
        plot(logged_data.data(:,logged_data.tagmap('t')),logged_data.data(:,logged_data.tagmap(tagname)))
        xlabel('t(s)')
        ylabel(tagname)
        grid on
    else
        error(['invalid tag name, the logged file called ' filename ' does not contain this tag!'])
    end
else
    error('invalid filename, the file does not contain logged_data!')
end
function varargout = GUI(varargin)
    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @GUI_OpeningFcn, ...
                       'gui_OutputFcn',  @GUI_OutputFcn, ...
                       'gui_LayoutFcn',  [] , ...
                       'gui_Callback',   []);
    if nargin && ischar(varargin{1})
        gui_State.gui_Callback = str2func(varargin{1});
    end

    if nargout
        [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
    else
        gui_mainfcn(gui_State, varargin{:});
    end
    % End initialization code - DO NOT EDIT


% --- Executes just before GUI is made visible.
function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to GUI (see VARARGIN)

    %INITIALIZATION
    %Defaults selectedRobot to the one the GUI places in the box at the
    %start
    contents = cellstr(get(handles.selectRobot,'String'));
    handles.selectedRobot = str2double(contents{get(handles.selectRobot,'Value')});

    %Defines colors for each of the robots
    colors = ['r' 'g' 'b' 'c' 'm' 'y'];
    for i=1:5
        handles.Robots(i).lineColor = colors(i);
    end

    %Can use this line to ask the user for a COM Port
    %x = inputdlg('Enter COM Port:');
    
    % Create the serial port object
    handles.serialPort = serial('COM7','BaudRate',9600,'DataBits',8,'Parity', 'None','StopBits',1);
    handles.serialPort.InputBufferSize = 1024;
    %handles.serialPort.BytesAvailableFcnCount = 7;
    %handles.serialPort.BytesAvailableFcnMode = 'byte';
    %handles.serialPort.BytesAvailableFcn = @instrcallback;

    fopen(handles.serialPort);
    %handles.serialPort.ReadAsyncMode = 'continuous';

    % Choose default command line output for GUI
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles) 
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;

% --- Executes on 'Start' button press.
function start_Callback(hObject, eventdata, handles)
    %Goes through each Robot
    for N=1:1

        % If the robot has a point it needs to go to this wont fail
        try
            isHasCommand = size(handles.Robots(N).x, 2);
        catch
            isHasCommand = 0;
        end

        % Check to see if the robot needs to be sent a command
        if isHasCommand > 0;
            % Plots it's initial position and defaults it's index to the
            % first in a series of waypoints, isHasCommand also happens to
            % be the number of waypoints the robot was given
            handles.Robots(N).currIndex = 1;
            handles.Robots(N).prevPlot = plot(handles.Robots(N).x(1), handles.Robots(N).y(1), '^r');
            handles.Robots(N).numOfWaypoints = isHasCommand;

            % Initialize the Robot using the autoCalibrate function on the
            % 3pi
            initialize = [2 N 0 0 3];
            LRC = CalculateCheckSum( initialize );
            checksumIndex = length(initialize) + 1;
            initialize(checksumIndex) = LRC;
            response = 21;
            
            % Send the initialize command to the robot
            while response ~= 6
                try
                    for j=1:length(initialize)
                        fwrite(handles.serialPort, [initialize(j)], 'uint8');
                        pause(.01);
                    end
                    response = fread(handles.serialPort, 1, 'uint8');
                 catch
                   disp('Error Sending Position!');
                end
            end
            % If the robot initialized then we can proceed in sending
            % waypoints
            if(response == 6)
                % Cycle through the way points
                for wayPoint=1:handles.Robots(N).numOfWaypoints-1
                    % Increase the index so we can keep up with where the
                    % robot is at in it's waypoint cycle
                    handles.Robots(N).currIndex = handles.Robots(N).currIndex + 1;
                    
                    % compile the gotoPoint command 
                    gotoPoint = [2, N, 2, 5, handles.Robots(N).x(handles.Robots(N).currIndex), handles.Robots(N).y(handles.Robots(N).currIndex), 3];
                    LRC = CalculateCheckSum( gotoPoint );
                    checksumIndex = length(gotoPoint) + 1;
                    gotoPoint(checksumIndex) = LRC;

                    % Send the gotoPoint command
                    response = 21;
                    while response ~= 6
                        try
                            for j=1:length(gotoPoint)
                                fwrite(handles.serialPort, gotoPoint(j), 'uint8');
                                pause(.01);
                            end
                            response = fread(handles.serialPort, 1, 'uint8');
                         catch
                           disp('Error Sending Position!');
                        end
                    end
                    
                    % If the gotoPoint command was succesful then sit and
                    % read the points as the robot traverses the grid
                    if(response == 6)
                        % Read the curPosition command that the robot sends
                        % up
                        response = fread(handles.serialPort, 8, 'uint8');
                        
                        % Get rid of the prevPlot and plot the new grid 
                        delete(handles.Robots(N).prevPlot);
                        handles.Robots(N).prevPlot = plot(response(5), response(6), '^r');  
                        drawnow
                        
                        % While the robot isn't at the waypoint we sent him
                        % to
                        while(response(5) ~= handles.Robots(N).x(handles.Robots(N).currIndex) || response(6) ~=  handles.Robots(N).y(handles.Robots(N).currIndex))
                            response = fread(handles.serialPort, 8, 'uint8');
                            % Plot the current position of the robot
                            if(response(4) == 6)
                                delete(handles.Robots(N).prevPlot);
                                handles.Robots(N).prevPlot = plot(response(5), response(6), 'or');
                                drawnow
                            end
                            %plot the obstacle the robot found
                            if(response(4) == 8)
                                 plot(response(5), response(6), 's',...
                                                                'MarkerEdgeColor','w',...
                                                                'MarkerFaceColor','r',...
                                                                'MarkerSize',10);
                                 drawnow         
                            end
                        end
                    end
                end
                for P=1:handles.Robots(N).numOfWaypoints
                    delete(handles.Robots(N).plot(P));
                end
            else
                disp('Error Robot Did Not Init.');        
            end
        end
    end
    guidata(hObject, handles);

    % --- Executes on button press in loadMatFile.
    function loadMatFile_Callback(hObject, eventdata, handles)
    % hObject    handle to loadMatFile (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    uiopen('load')
    hold on
    % if the file says there are multple robots import the arrays
    if(exist('MultiRobot', 'var') == 1)
        for N=1:MultiRobot
            handles.Robots(N).x = x(N, 1:size(x,1));
            handles.Robots(N).y = y(N, 1:size(y,1));
            plot(handles.Robots(N).x, handles.Robots(N).y, handles.Robots(N).lineColor, 'LineWidth', 3);
        end
    else
        handles.Robots(handles.selectedRobot).x = x;
        handles.Robots(handles.selectedRobot).y = y;
        plot(x, y, handles.Robots(handles.selectedRobot).lineColor, 'LineWidth', 3);
    end

    guidata(hObject, handles);

% --- Executes on button press in cursorSelect.
function cursorSelect_Callback(hObject, eventdata, handles)

%Allow the user to select waypoints manually
hold on
i=1;
button = 1;
%Clear out the old
handles.Robots(handles.selectedRobot).x = [];
handles.Robots(handles.selectedRobot).y = [];
%while you don't rightclick
while button ~= 3
    %add new points to your waypoint array
    [x,y, button]=ginput(1);
    if(button ~= 3)
        handles.Robots(handles.selectedRobot).x(i)=round(x);
        handles.Robots(handles.selectedRobot).y(i)=round(y);
        handles.Robots(handles.selectedRobot).plot(i) = plot(handles.Robots(handles.selectedRobot).x, handles.Robots(handles.selectedRobot).y, handles.Robots(handles.selectedRobot).lineColor,'LineWidth',3);
        i = i+1;
    end
end

guidata(hObject, handles);

% --- Executes on selection change in selectRobot.
function selectRobot_Callback(hObject, eventdata, handles)
% Changes which robot you are selecting waypoints for
contents = cellstr(get(hObject,'String'));
handles.selectedRobot = str2double(contents{get(hObject,'Value')});
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function selectRobot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to selectRobot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function robotChatter_Callback(hObject, eventdata, handles)
% hObject    handle to robotChatter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of robotChatter as text
%        str2double(get(hObject,'String')) returns contents of robotChatter as a double

%TODO

% --- Executes during object creation, after setting all properties.
function robotChatter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to robotChatter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(hObject, eventdata, handles)
% If you close the figure, kill the serial ports
fclose(handles.serialPort);
delete(handles.serialPort);

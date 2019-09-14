function varargout = gui(varargin)
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_OutputFcn, ...
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

function gui_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

%Hide plot axes
axes(handles.axes1);
set(gca,'visible','off');
axes(handles.axes2);
set(gca,'visible','off');

%Disable push buttons and slider
set(findall(handles.pushbutton2, '-property', 'enable'), 'enable', 'off');
set(findall(handles.pushbutton3, '-property', 'enable'), 'enable', 'off');
set(findall(handles.pushbutton4, '-property', 'enable'), 'enable', 'off');
set(findall(handles.pushbutton5, '-property', 'enable'), 'enable', 'off');
set(findall(handles.slider1, '-property', 'enable'), 'enable', 'off');
set(findall(handles.slider2, '-property', 'enable'), 'enable', 'off');
set(handles.edit2,'string',num2str(get(handles.slider1,'Value')));
set(handles.edit3,'string',num2str(get(handles.slider2,'Value')));

function varargout = gui_OutputFcn(hObject, eventdata, handles) 
varargout{1} = handles.output;

function slider1_Callback(hObject, eventdata, handles)
set(handles.edit2,'string',num2str(get(handles.slider1,'Value')));

function slider1_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function popupmenu1_Callback(hObject, eventdata, handles)

function popupmenu1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function popupmenu2_Callback(hObject, eventdata, handles)

function popupmenu2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function pushbutton1_Callback(hObject, eventdata, handles)
serial_ports = seriallist;
if (~ isempty(serial_ports))
    set(handles.popupmenu2,'string',serial_ports)
    set(findall(handles.pushbutton2, '-property', 'enable'), 'enable', 'on');
end

function pushbutton2_Callback(hObject, eventdata, handles)
%Disable update ports and start buttons
set(findall(handles.pushbutton1, '-property', 'enable'), 'enable', 'off');
set(findall(handles.pushbutton2, '-property', 'enable'), 'enable', 'off');

%Open serial connection with selected serial port and baud rate
baud_rate_list = get(handles.popupmenu1,'String');
serial_port_list = get(handles.popupmenu2,'String');
baud_rate = str2double(baud_rate_list(get(handles.popupmenu1,'Value')));
serial_size = size(serial_port_list);
if (serial_size(1,1) == 1) 
    serial_port = serial_port_list;
else
    serial_port = char(serial_port_list(get(handles.popupmenu2,'Value')));
end
global s;
s = serial(serial_port);
set(s ,'BaudRate', baud_rate);
fopen(s);
pause(2);

%Enable send and close buttons and slider 1
set(findall(handles.pushbutton3, '-property', 'enable'), 'enable', 'on');
set(findall(handles.pushbutton4, '-property', 'enable'), 'enable', 'on');
set(findall(handles.pushbutton5, '-property', 'enable'), 'enable', 'on');
set(findall(handles.slider1, '-property', 'enable'), 'enable', 'on');
set(findall(handles.slider2, '-property', 'enable'), 'enable', 'on');

%Receive number of nodes
num_nodes = fread(s, 1, 'uint8');
%disp(num_nodes)

%Receive external illuminance
ext_illum_array(1) = fread(s, 1, 'uint8');
ext_illum_array(2) = fread(s, 1, 'uint8');
ext_illum = typecast(uint8(ext_illum_array), 'int16');
ext_illum = double(swapbytes(ext_illum))/100;
set(handles.edit6,'string',num2str(ext_illum));
%disp(ext_illum_array);

%Receive gain
for i=1:num_nodes
    gain_array(1) = fread(s, 1, 'uint8');
    gain_array(2) = fread(s, 1, 'uint8');
    gain = typecast(uint8(gain_array), 'int16');
    k(i) = double(swapbytes(gain))/100;
end
set(handles.edit1,'string', num2str(k));

%Receive gains
% aux_str = strsplit(fscanf(s),',');
% aux_str = strcat(aux_str(1),',', aux_str(2));
% set(handles.edit1,'string',aux_str);

%Sets timer for receving serial data
t = timer();
t.Period = 0.75;
t.ExecutionMode = 'fixedRate';
handles.timer = t;
set(handles.timer, 'TimerFcn', {@timer_event, hObject});
start(handles.timer)
guidata(hObject, handles);

%Show plot axes
axes(handles.axes1);
set(gca,'visible','on');
axes(handles.axes2);
set(gca,'visible','on');

%Init global arrays
global illuminance_array;
global voltage_array;
illuminance_array = zeros(1, 15);
voltage_array = zeros(1, 15);

function timer_event(hObject, eventData, parent_GUI)
%tic
global s;
global illuminance_array;
global voltage_array;
handles = guidata(parent_GUI);

if(s.BytesAvailable > 0)
    
    %aux_str = strsplit(fscanf(s),',');
    
    for i=1:4
        read_array(1) = fread(s, 1, 'uint8');
        read_array(2) = fread(s, 1, 'uint8');
        gain = typecast(uint8(read_array), 'int16');
        read(i) = double(swapbytes(gain))/100;
    end
       
    %Update arrays
    voltage_array(1:14) = voltage_array(2:15);
    voltage_array(15) = read(2);
    illuminance_array(1:14) = illuminance_array(2:15);
    illuminance_array(15) = read(1);

    %LDR Voltage
    plot(handles.axes2, voltage_array);
    xlabel(handles.axes2,'Point');
    ylabel(handles.axes2,'v[V]');
    title(handles.axes2,'LED actuation voltage');

    %Illuminance plot
    plot(handles.axes1, illuminance_array);
    xlabel(handles.axes1, 'Point');
    ylabel(handles.axes1, 'Y[LUX]');
    title(handles.axes1,'Illuminance in LDR');
    
    %Distributed controller values
    set(handles.edit4,'string', num2str(read(3)));
    set(handles.edit5,'string', num2str(read(4)));
    
    %disp(aux_str(5));
end
%toc

function pushbutton4_Callback(hObject, eventdata, handles)
%Stops timer and closes serial communication
stop(handles.timer)
delete(handles.timer)
global s
status = get(s,{'Status'});
if (strcmp(status, "open"))
    fclose(s);
    delete(s);
    clear s;
end
close

function pushbutton3_Callback(hObject, eventdata, handles)
global s;
message_value = uint8(get(handles.slider1,'Value'));
fwrite(s, 0);
%disp(0);
fwrite(s, message_value);
%disp(message_value);

%message_value = num2str(get(handles.slider1,'Value'));
%message = strcat("reference",',', message_value);
%disp(message);
%fprintf(s, message);

function edit1_Callback(hObject, eventdata, handles)

function edit1_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit2_Callback(hObject, eventdata, handles)

function edit2_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function pushbutton5_Callback(hObject, eventdata, handles)
global s;
message_value = uint8(get(handles.slider2,'Value'));
fwrite(s, 1);
%disp(1);
fwrite(s, message_value);
%disp(message_value);

%message_value = num2str(get(handles.slider2,'Value'));
%message = strcat("cost",',', message_value);
%disp(message);
%fprintf(s, message);

function slider2_Callback(hObject, eventdata, handles)
set(handles.edit3,'string',num2str(get(handles.slider2,'Value')));

function slider2_CreateFcn(hObject, eventdata, handles)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit3_Callback(hObject, eventdata, handles)

function edit3_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit4_Callback(hObject, eventdata, handles)

function edit4_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit5_Callback(hObject, eventdata, handles)

function edit5_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit6_Callback(hObject, eventdata, handles)

function edit6_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

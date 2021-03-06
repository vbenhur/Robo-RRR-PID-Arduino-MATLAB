function varargout = serial_GUI(varargin)
%  	Author: Roger Yeh
%   Copyright 2010 MathWorks, Inc.
%   Version: 1.0  |  Date: 2010.01.13

% SERIAL_GUI M-file for serial_GUI.fig
%      SERIAL_GUI, by itself, creates a new SERIAL_GUI or raises the existing
%      singleton*.
%
%      H = SERIAL_GUI returns the handle to a new SERIAL_GUI or the handle to
%      the existing singleton*.
%
%      SERIAL_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SERIAL_GUI.M with the given input arguments.
%
%      SERIAL_GUI('Property','Value',...) creates a new SERIAL_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before serial_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to serial_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help serial_GUI

% Last Modified by GUIDE v2.5 06-Nov-2015 16:45:06

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @serial_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @serial_GUI_OutputFcn, ...
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


% --- Executes just before serial_GUI is made visible.
function serial_GUI_OpeningFcn(hObject, eventdata, handles, varargin)

serialPorts = instrhwinfo('serial');
nPorts = length(serialPorts.SerialPorts);
set(handles.portList, 'String', ...
    [{'Select a port'} ; serialPorts.SerialPorts ]);
set(handles.portList, 'Value', 2);   
set(handles.history_box, 'String', cell(1));

handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes serial_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = serial_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in portList.
function portList_Callback(hObject, eventdata, handles)
% hObject    handle to portList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns portList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from portList


% --- Executes during object creation, after setting all properties.
function portList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to portList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function history_box_Callback(hObject, eventdata, handles)
% hObject    handle to history_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of history_box as text
%        str2double(get(hObject,'String')) returns contents of history_box as a double


% --- Executes during object creation, after setting all properties.
function history_box_CreateFcn(hObject, eventdata, handles)
% hObject    handle to history_box (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Tx_send_Callback(hObject, eventdata, handles)
TxText = get(handles.Tx_send, 'String');
fprintf(handles.serConn, TxText);

currList = get(handles.history_box, 'String');

set(handles.history_box, 'String', ...
    [currList ; ['Sent @ ' datestr(now) ': ' TxText] ]);
set(handles.history_box, 'Value', length(currList) + 1 );

set(hObject, 'String', '');



% --- Executes during object creation, after setting all properties.
function Tx_send_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Tx_send (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in rxButton.
function rxButton_Callback(hObject, eventdata, handles)
try 
    RxText = fscanf(handles.serConn);
    currList = get(handles.history_box, 'String');
    if length(RxText) < 1
        RxText = 'Timeout @ ';
        set(handles.history_box, 'String', ...
            [currList ; [RxText datestr(now)] ]);
    else
        set(handles.history_box, 'String', ...
            [currList ; ['Received @ ' datestr(now) ': ' RxText ] ]);
    end
    set(handles.history_box, 'Value', length(currList) + 1 );
catch e
    disp(e)
end

function baudRateText_Callback(hObject, eventdata, handles)
% hObject    handle to baudRateText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of baudRateText as text
%        str2double(get(hObject,'String')) returns contents of baudRateText as a double


% --- Executes during object creation, after setting all properties.
function baudRateText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to baudRateText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in connectButton.
function connectButton_Callback(hObject, eventdata, handles)    
if strcmp(get(hObject,'String'),'Connect') % currently disconnected
    serPortn = get(handles.portList, 'Value');
    if serPortn == 1
        errordlg('Select valid COM port');
    else
        serList = get(handles.portList,'String');
        serPort = serList{serPortn};
        serConn = serial(serPort, 'TimeOut', 1, ...
            'BaudRate', str2num(get(handles.baudRateText, 'String')));
        
        try
            fopen(serConn);
            handles.serConn = serConn;
            
            % enable Tx text field and Rx button
            set(handles.EditBase, 'Enable', 'On');
            set(handles.sliderBase, 'Enable', 'On');
            set(handles.EditOmbro, 'Enable', 'On');
            set(handles.EditCotovelo, 'Enable', 'On');
            set(handles.EditPulso, 'Enable', 'On');
            set(handles.rxButton, 'Enable', 'On');
            
            set(hObject, 'String','Disconnect')
        catch e
            errordlg(e.message);
        end
        
    end
else
    set(handles.EditBase, 'Enable', 'Off');
     set(handles.sliderBase, 'Enable', 'Off');
    set(handles.EditOmbro, 'Enable', 'Off');
    set(handles.EditCotovelo, 'Enable', 'Off');
    set(handles.EditPulso, 'Enable', 'On');
    set(handles.rxButton, 'Enable', 'Off');
    
    set(hObject, 'String','Connect')
    fclose(handles.serConn);
end
guidata(hObject, handles);


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if isfield(handles, 'serConn')
    fclose(handles.serConn);
end
% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on key press with focus on Tx_send and none of its controls.
function Tx_send_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to Tx_send (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over Tx_send.
function Tx_send_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to Tx_send (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object deletion, before destroying properties.
function Tx_send_DeleteFcn(hObject, eventdata, handles)
% hObject    handle to Tx_send (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function EditBase_Callback(hObject, eventdata, handles)
% hObject    handle to EditBase (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EditBase as text
%        str2double(get(hObject,'String')) returns contents of EditBase as a double
valBase = str2double(get(hObject,'String'));
set(handles.sliderBase,'Value',valBase);

%TxtBase = get(handles.EditBase, 'String');
%fprintf(handles.serConn, TxtBase);

%currList = get(handles.history_box, 'String');

%set(handles.history_box, 'String', ...
%    [currList ; ['Sent @ ' datestr(now) ': ' TxtBase] ]);
%set(handles.history_box, 'Value', length(currList) + 1 );

% --- Executes during object creation, after setting all properties.
function EditBase_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EditBase (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on key press with focus on EditBase and none of its controls.
function EditBase_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to EditBase (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushButton4.
function pushButton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushButton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%TxtBase = get(handles.EditBase, 'String');
TxtBase = get(handles.EditBase, 'String');
TxtOmbro = get(handles.EditOmbro, 'String');
TxtCotovelo = get(handles.EditCotovelo, 'String');
TxtPulso = get(handles.EditPulso, 'String');
%fprintf(handles.serConn, '{"Dados":{"B":'+TxtBase+',"O":'+TxtOmbro+',"C":'+TxtCotovelo+',"P":'+TxtBase+',"G":'+TxtBase+'}}');

C = {'{"Dados":{"B":',',"O":',',"C":',',"P":',',"G":','}}'};
str = strjoin(C,{TxtBase, TxtOmbro, TxtCotovelo, TxtPulso,'0' })
fprintf(handles.serConn, str);


function EditOmbro_Callback(hObject, eventdata, handles)
% hObject    handle to EditOmbro (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EditBase as text
%        str2double(get(hObject,'String')) returns contents of EditBase as a double


% --- Executes during object creation, after setting all properties.
function EditOmbro_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EditOmbro (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function EditCotovelo_Callback(hObject, eventdata, handles)
% hObject    handle to EditCotovelo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EditCotovelo as text
%        str2double(get(hObject,'String')) returns contents of EditCotovelo as a double


% --- Executes during object creation, after setting all properties.
function EditCotovelo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EditCotovelo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function EditPulso_Callback(hObject, eventdata, handles)
% hObject    handle to EditPulso (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of EditBase as text
%        str2double(get(hObject,'String')) returns contents of EditBase as a double


% --- Executes during object creation, after setting all properties.
function EditPulso_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EditPulso (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function sliderBase_Callback(hObject, eventdata, handles)
% hObject    handle to sliderBase (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
valBase = get(hObject,'Value');
set(handles.EditBase,'String',num2str(valBase));


% --- Executes during object creation, after setting all properties.
function sliderBase_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sliderBase (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in btFechaGarra.
function btFechaGarra_Callback(hObject, eventdata, handles)
% hObject    handle to btFechaGarra (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global txtGarra;
txtGarra = '2';
C = {'{"Dados":{"G":','}}'};
str = strjoin(C,{txtGarra})
fprintf(handles.serConn, str);
str = strjoin(C,{'0'})
fprintf(handles.serConn, str);


% --- Executes on button press in btAbrirGarra.
function btAbrirGarra_Callback(hObject, eventdata, handles)
% hObject    handle to btAbrirGarra (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global txtGarra;
txtGarra = '1';
C = {'{"Dados":{"G":','}}'};
str = strjoin(C,{txtGarra})
fprintf(handles.serConn, str);

str = strjoin(C,{'0'})
fprintf(handles.serConn, str);



% --- Executes on key press with focus on btAbrirGarra and none of its controls.
function btAbrirGarra_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to btAbrirGarra (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over btAbrirGarra.
function btAbrirGarra_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to btAbrirGarra (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp('bt apertado')


% --- Executes on key press with focus on rxButton and none of its controls.
function rxButton_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to rxButton (see GCBO)
% eventdata  structure with the following fields (see UICONTROL)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

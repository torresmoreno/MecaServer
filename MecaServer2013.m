function varargout = MecaServer2013(varargin)
% MECASERVER MATLAB code for MecaServer.fig
%      MECASERVER, by itself, creates a new MECASERVER or raises the existing
%      singleton*.
%
%      H = MECASERVER returns the handle to a new MECASERVER or the handle to
%      the existing singleton*.
%
%      MECASERVER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MECASERVER.M with the given input arguments.
%
%      MECASERVER('Property','Value',...) creates a new MECASERVER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MecaServer_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MecaServer_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MecaServer

% Last Modified by GUIDE v2.5 29-Aug-2016 13:47:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @MecaServer_OpeningFcn, ...
    'gui_OutputFcn',  @MecaServer_OutputFcn, ...
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


% --- Executes just before MecaServer is made visible.
function MecaServer_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MecaServer (see VARARGIN)

% Choose default command line output for MecaServer
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes MecaServer wait for user response (see UIRESUME)
% uiwait(handles.figure1);
resetear(handles)
% Determine where your m-file's folder is.
folder = fileparts(which('MecaServer')); 
% Add that folder plus all subfolders to the path.
addpath(genpath(folder));
addpath projects
% --- Outputs from this function are returned to the command line.
function varargout = MecaServer_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in botonNuevo.
function botonNuevo_Callback(hObject, eventdata, handles)
% hObject    handle to botonNuevo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
resetear(handles)
resetear_soluciones(handles)
set(handles.tablaCoord,'ColumnEditable',true(1,3));


% --- Executes on button press in botonCargar.
function botonCargar_Callback(hObject, eventdata, handles)
% hObject    handle to botonCargar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
resetear_soluciones(handles)
nombre=get(handles.editNombre,'String');
eval(sprintf('load %s', nombre))

% Escritura de los parametros del problema
set(handles.editBeta,'String',datos.beta);
set(handles.editS,'String',datos.s);
set(handles.editL2,'String',datos.L2);
set(handles.editL3,'String',datos.L3);
set(handles.editL4,'String',datos.L4);
set(handles.editPosGDL,'String',datos.posGDL);
set(handles.editVelGDL,'String',datos.velGDL);
set(handles.editAceGDL,'String',datos.aceGDL);
set(handles.editDuracion,'String',datos.duracion);
set(handles.popupTipo,'Value',datos.tipo);
set(handles.tablaCoord,'Data',datos.coord);


% --- Executes on button press in botonGuardar.
function botonGuardar_Callback(hObject, eventdata, handles)
% hObject    handle to botonGuardar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
datos=leer(handles);
warndlg('Se sobreescribiran los datos','Cuidado!')
eval(sprintf('save projects/%s datos', datos.nombre))


function editNombre_Callback(hObject, eventdata, handles)
% hObject    handle to editNombre (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editNombre as text
%        str2double(get(hObject,'String')) returns contents of editNombre as a double


% --- Executes during object creation, after setting all properties.
function editNombre_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editNombre (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupTipo.
function popupTipo_Callback(hObject, eventdata, handles)
% hObject    handle to popupTipo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupTipo contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupTipo

switch get(handles.popupTipo,'Value')
    case 2
        %imageview('BielaBalancin.png')
    case 3
        %imageview('imagen2.png')
    case 4
        %imageview('imagen2.png')
    case 5
        %imageview('imagen2.png')
end

% --- Executes during object creation, after setting all properties.
function popupTipo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupTipo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editPosGDL_Callback(hObject, eventdata, handles)
% hObject    handle to editPosGDL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editPosGDL as text
%        str2double(get(hObject,'String')) returns contents of editPosGDL as a double


% --- Executes during object creation, after setting all properties.
function editPosGDL_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editPosGDL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editVelGDL_Callback(hObject, eventdata, handles)
% hObject    handle to editVelGDL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editVelGDL as text
%        str2double(get(hObject,'String')) returns contents of editVelGDL as a double


% --- Executes during object creation, after setting all properties.
function editVelGDL_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editVelGDL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editAceGDL_Callback(hObject, eventdata, handles)
% hObject    handle to editAceGDL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAceGDL as text
%        str2double(get(hObject,'String')) returns contents of editAceGDL as a double


% --- Executes during object creation, after setting all properties.
function editAceGDL_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAceGDL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in botonSolPos.
function botonSolPos_Callback(hObject, eventdata, handles)
% hObject    handle to botonSolPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

mbs=modelo(handles);

q=mbeKinematicsSolver.pos_problem(mbs,mbs.q_init_aprox);
resultados(handles,mbs,q)
axes(handles.axes1)
pruebadibuja(mbs.mechanism_type,[q;mbs.fixed_points']);


% --- Executes on button press in botonSolVel.
function botonSolVel_Callback(hObject, eventdata, handles)
% hObject    handle to botonSolVel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

mbs=modelo(handles);

q=mbeKinematicsSolver.pos_problem(mbs,mbs.q_init_aprox);
qp=mbeKinematicsSolver.vel_problem(mbs,q,mbs.zp_init);
resultados(handles,mbs,q,qp)
axes(handles.axes1)
pruebadibuja(mbs.mechanism_type,[q;mbs.fixed_points'],qp);


% --- Executes on button press in botonSolAce.
function botonSolAce_Callback(hObject, eventdata, handles)
% hObject    handle to botonSolAce (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

mbs=modelo(handles);

q=mbeKinematicsSolver.pos_problem(mbs,mbs.q_init_aprox);
qp=mbeKinematicsSolver.vel_problem(mbs,q,mbs.zp_init);
qpp=mbeKinematicsSolver.accel_problem(mbs,q,qp,mbs.zpp_init);
resultados(handles,mbs,q,qp,qpp)
axes(handles.axes1)
pruebadibuja(mbs.mechanism_type,[q;mbs.fixed_points'],qp,qpp);


function editDuracion_Callback(hObject, eventdata, handles)
% hObject    handle to editDuracion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editDuracion as text
%        str2double(get(hObject,'String')) returns contents of editDuracion as a double


% --- Executes during object creation, after setting all properties.
function editDuracion_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editDuracion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in botonSimular.
function botonSimular_Callback(hObject, eventdata, handles)
% hObject    handle to botonSimular (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%addpath ../dibujar
axes(handles.axes1)
mbs=modelo(handles);

q=mbeKinematicsSolver.pos_problem(mbs,mbs.q_init_aprox);
qp=mbeKinematicsSolver.vel_problem(mbs,q,mbs.zp_init);
qpp=mbeKinematicsSolver.accel_problem(mbs,q,qp,mbs.zpp_init);

duracion = str2double(get(handles.editDuracion,'String'));
At=0.005;
pasos = duracion/At;

t = zeros(pasos,1);
fichero = zeros(pasos, 1+3*length(q));
i=0;


q=mbeKinematicsSolver.pos_problem(mbs,mbs.q_init_aprox);
while i<pasos
    var = mbs.indep_idxs;    
    i=i+1;
    t(i)= i*At;
    q(var) = mbs.q_init_aprox(var)+...
        (t(i)*qp(var)+...
        t(i)^2*0.5*mbs.zpp_init);
    qp(var) = qp(var)+At*mbs.zpp_init;
    
    q=mbeKinematicsSolver.pos_problem(mbs,q);
    qp=mbeKinematicsSolver.vel_problem(mbs,q,qp(var));
    qpp=mbeKinematicsSolver.accel_problem(mbs,q,qp,qpp(var));
    fichero(i,:)=[t(i), q', qp', qpp'];
    resultados(handles,mbs,q,qp,qpp)
    pruebadibuja(mbs.mechanism_type,[q;mbs.fixed_points'],qp,qpp);
    title(num2str(t(i)))
end

nombre=get(handles.editNombre,'String');
nombreextra = '_resultados';
eval(sprintf('save projects/%s fichero', strcat(nombre,nombreextra)))




function editVx1_Callback(hObject, eventdata, handles)
% hObject    handle to editVx1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editVx1 as text
%        str2double(get(hObject,'String')) returns contents of editVx1 as a double


% --- Executes during object creation, after setting all properties.
function editVx1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editVx1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editVy1_Callback(hObject, eventdata, handles)
% hObject    handle to editVy1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editVy1 as text
%        str2double(get(hObject,'String')) returns contents of editVy1 as a double


% --- Executes during object creation, after setting all properties.
function editVy1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editVy1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editModV1_Callback(hObject, eventdata, handles)
% hObject    handle to editModV1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editModV1 as text
%        str2double(get(hObject,'String')) returns contents of editModV1 as a double


% --- Executes during object creation, after setting all properties.
function editModV1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editModV1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editDirV1_Callback(hObject, eventdata, handles)
% hObject    handle to editDirV1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editDirV1 as text
%        str2double(get(hObject,'String')) returns contents of editDirV1 as a double


% --- Executes during object creation, after setting all properties.
function editDirV1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editDirV1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editAx1_Callback(hObject, eventdata, handles)
% hObject    handle to editAx1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAx1 as text
%        str2double(get(hObject,'String')) returns contents of editAx1 as a double


% --- Executes during object creation, after setting all properties.
function editAx1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAx1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editAy1_Callback(hObject, eventdata, handles)
% hObject    handle to editAy1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAy1 as text
%        str2double(get(hObject,'String')) returns contents of editAy1 as a double


% --- Executes during object creation, after setting all properties.
function editAy1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAy1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editModA1_Callback(hObject, eventdata, handles)
% hObject    handle to editModA1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editModA1 as text
%        str2double(get(hObject,'String')) returns contents of editModA1 as a double


% --- Executes during object creation, after setting all properties.
function editModA1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editModA1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editDirA1_Callback(hObject, eventdata, handles)
% hObject    handle to editDirA1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editDirA1 as text
%        str2double(get(hObject,'String')) returns contents of editDirA1 as a double


% --- Executes during object creation, after setting all properties.
function editDirA1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editDirA1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editVx2_Callback(hObject, eventdata, handles)
% hObject    handle to editVx2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editVx2 as text
%        str2double(get(hObject,'String')) returns contents of editVx2 as a double


% --- Executes during object creation, after setting all properties.
function editVx2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editVx2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editVy2_Callback(hObject, eventdata, handles)
% hObject    handle to editVy2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editVy2 as text
%        str2double(get(hObject,'String')) returns contents of editVy2 as a double


% --- Executes during object creation, after setting all properties.
function editVy2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editVy2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editModV2_Callback(hObject, eventdata, handles)
% hObject    handle to editModV2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editModV2 as text
%        str2double(get(hObject,'String')) returns contents of editModV2 as a double


% --- Executes during object creation, after setting all properties.
function editModV2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editModV2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editDirV2_Callback(hObject, eventdata, handles)
% hObject    handle to editDirV2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editDirV2 as text
%        str2double(get(hObject,'String')) returns contents of editDirV2 as a double


% --- Executes during object creation, after setting all properties.
function editDirV2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editDirV2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editAx2_Callback(hObject, eventdata, handles)
% hObject    handle to editAx2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAx2 as text
%        str2double(get(hObject,'String')) returns contents of editAx2 as a double


% --- Executes during object creation, after setting all properties.
function editAx2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAx2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editAy2_Callback(hObject, eventdata, handles)
% hObject    handle to editAy2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editAy2 as text
%        str2double(get(hObject,'String')) returns contents of editAy2 as a double


% --- Executes during object creation, after setting all properties.
function editAy2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editAy2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editModA2_Callback(hObject, eventdata, handles)
% hObject    handle to editModA2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editModA2 as text
%        str2double(get(hObject,'String')) returns contents of editModA2 as a double


% --- Executes during object creation, after setting all properties.
function editModA2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editModA2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editDirA2_Callback(hObject, eventdata, handles)
% hObject    handle to editDirA2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editDirA2 as text
%        str2double(get(hObject,'String')) returns contents of editDirA2 as a double


% --- Executes during object creation, after setting all properties.
function editDirA2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editDirA2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in botonLog.
function botonLog_Callback(hObject, eventdata, handles)
% hObject    handle to botonLog (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

concatenado = strcat(get(handles.editNombre,'String'),'_resultados');
eval(sprintf('load %s', concatenado))
%cadena_concatenada = strcat('Para cargar el fichero de resultados, ejecute: load ', ' :load', concatenado);
helpdlg(sprintf('Para cargar el fichero de resultados en el Workspace, ejecute: load %s\n', concatenado));

% --- Executes on button press in botonPlot.
function botonPlot_Callback(hObject, eventdata, handles)
% hObject    handle to botonPlot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
mbs=modelo(handles);
concatenado = strcat(get(handles.editNombre,'String'),'_resultados');
eval(sprintf('load %s', concatenado))

lq = length(mbs.q_init_aprox);

figure('Name','Resultados de la simulacion')
subplot(3,1,1)

switch mbs.mechanism_type
    case {1,3} % Cuatro barras
        plotyy(fichero(:,1),rad2deg(fichero(:,lq)),fichero(:,1),rad2deg(fichero(:,lq+1)))
        title('Relacion entrada (azul) y la salida (verde)')
        ylabel('\theta (deg)')               
        
        subplot(3,1,2)
        plotyy(fichero(:,1),fichero(:,2*lq),fichero(:,1),fichero(:,2*lq+1))
        ylabel('\omega (rad/s)')
       
        subplot(3,1,3)
        plotyy(fichero(:,1),fichero(:,3*lq),fichero(:,1),fichero(:,3*lq+1))
        ylabel('\alpha (rad/s^2)')        
        
    case 2 % Biela-manivela
        plotyy(fichero(:,1),rad2deg(fichero(:,lq)),fichero(:,1),fichero(:,lq+1))
        title('Relacion entrada (azul) y la salida (verde)')
        ylabel('\theta (deg) - dist')
        
        subplot(3,1,2)
        plotyy(fichero(:,1),fichero(:,2*lq),fichero(:,1),fichero(:,2*lq+1))
        ylabel('\omega (rad/s) - vel')       
        
        subplot(3,1,3)
        plotyy(fichero(:,1),fichero(:,3*lq),fichero(:,1),fichero(:,3*lq+1))
        ylabel('\alpha (rad/s^2) - ace')  
          
end
xlabel('t (s)')

function editL2_Callback(hObject, eventdata, handles)
% hObject    handle to editL2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editL2 as text
%        str2double(get(hObject,'String')) returns contents of editL2 as a double


% --- Executes during object creation, after setting all properties.
function editL2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editL2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editL3_Callback(hObject, eventdata, handles)
% hObject    handle to editL3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editL3 as text
%        str2double(get(hObject,'String')) returns contents of editL3 as a double


% --- Executes during object creation, after setting all properties.
function editL3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editL3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editL4_Callback(hObject, eventdata, handles)
% hObject    handle to editL4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editL4 as text
%        str2double(get(hObject,'String')) returns contents of editL4 as a double


% --- Executes during object creation, after setting all properties.
function editL4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editL4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editBeta_Callback(hObject, eventdata, handles)
% hObject    handle to editBeta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editBeta as text
%        str2double(get(hObject,'String')) returns contents of editBeta as a double


% --- Executes during object creation, after setting all properties.
function editBeta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editBeta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editS_Callback(hObject, eventdata, handles)
% hObject    handle to editS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editS as text
%        str2double(get(hObject,'String')) returns contents of editS as a double


% --- Executes during object creation, after setting all properties.
function editS_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editS (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in botonReset.
function botonReset_Callback(hObject, eventdata, handles)
% hObject    handle to botonReset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close all
MecaServer
%=====================================================================
% Mis funciones
%=====================================================================



function [] = resetear(handles)
% Escritura de los parametros del problema
set(handles.editBeta,'String',0);
set(handles.editS,'String',0);
set(handles.editL2,'String',0);
set(handles.editL3,'String',0);
set(handles.editL4,'String',0);
set(handles.editPosGDL,'String',0);
set(handles.editVelGDL,'String',0);
set(handles.editAceGDL,'String',0);
%set(handles.popupTipo,'Value',1);
set(handles.editDuracion,'String',0);
set(handles.tablaCoord,'Data',zeros(5,2));


function [] = resetear_soluciones(handles)
% Rotaciones
rotaciones = zeros(3,3);
set(handles.tablaRot,'Data',rotaciones);

% Velocidades
set(handles.editVelGDL,'String',0)
set(handles.editVx1,'String',0);
set(handles.editVy1,'String',0);
set(handles.editModV1,'String',0);
set(handles.editVx2,'String',0);
set(handles.editVy2,'String',0);
set(handles.editModV2,'String',0);
set(handles.editDirV1,'String',0);
set(handles.editDirV2,'String',0);

% Aceleraciones
set(handles.editAx1,'String',0);
set(handles.editAy1,'String',0);
set(handles.editModA1,'String',0);
set(handles.editAx2,'String',0);
set(handles.editAy2,'String',0);
set(handles.editModA2,'String',0);
set(handles.editDirA1,'String',0);
set(handles.editDirA2,'String',0);
function [datos] = leer(handles)
% Escritura de los parametros del problema
datos.beta=str2double(get(handles.editBeta,'String'));
datos.s=str2double(get(handles.editS,'String'));
datos.L2=str2double(get(handles.editL2,'String'));
datos.L3=str2double(get(handles.editL3,'String'));
datos.L4=str2double(get(handles.editL4,'String'));
datos.posGDL=str2double(get(handles.editPosGDL,'String'));
datos.velGDL=str2double(get(handles.editVelGDL,'String'));
datos.aceGDL=str2double(get(handles.editAceGDL,'String'));
datos.nombre=get(handles.editNombre,'String');
datos.duracion=str2double(get(handles.editDuracion,'String'));
datos.coord=get(handles.tablaCoord,'Data');
datos.tipo=get(handles.popupTipo,'Value');

function mbs=modelo(handles)
datos = leer(handles);
xA=datos.coord(1,1); yA=datos.coord(1,2);
x1=datos.coord(2,1); y1=datos.coord(2,2);
x2=datos.coord(3,1); y2=datos.coord(3,2);
xB=datos.coord(4,1); yB=datos.coord(4,2);
switch datos.tipo
    case 2 % Cuatro barras
        % Crear objeto y configurarlo
        mbs = mbeMechModelFourBars4();
        mbs.q_init_aprox=[x1 y1 x2 y2 datos.posGDL datos.beta]';
        mbs.bar_lengths = [datos.L2, datos.L3, datos.L4];
        mbs.fixed_points = [xA, yA, xB, yB];
    case 3 % Biela manivela
        % Crear objeto y configurarlo
        mbs = mbeMechModelSliderCrank();
        mbs.q_init_aprox=[x1 y1 x2 y2 datos.posGDL datos.s]';
        xC=datos.coord(5,1); yC=datos.coord(5,2);
        mbs.fixed_points = [xA, yA, xB, yB, xC, yC];
        mbs.xC = xC; mbs.yC = yC;
        mbs.bar_lengths = [datos.L2, datos.L3];
        
    case 4 % Coriolis
        % Crear objeto y configurarlo
        mbs = mbeMechModelCoriolis();
        mbs.q_init_aprox=[x1 y1 x2 y2 datos.posGDL datos.beta]';
        mbs.fixed_points = [xA, yA, xB, yB];
        mbs.bar_lengths = [datos.L2, datos.L3];
end
mbs.xA = xA; mbs.yA = yA;
mbs.xB = xB; mbs.yB = yB;
mbs.zp_init = datos.velGDL;
mbs.zpp_init = datos.aceGDL;

function []=pruebadibuja(tipo,q,varargin)
switch tipo
    case 1 % Cuatro Barras
        if (nargin==2)
            dibujaCuatroBarras(q)
        end
        if (nargin==3)
            dq = varargin{1};
            dibujaCuatroBarras(q,dq)
        end
        if (nargin>=4)
            dq = varargin{1};
            ddq = varargin{2};
            dibujaCuatroBarras(q,dq,ddq)
        end
    case 2 % Biela Manivela
        if (nargin==2)
            dibujaSliderCrank(q)
        end
        if (nargin>=3)
            dq = varargin{1};
            dibujaSliderCrank(q,dq)
        end
        if (nargin>=4)
            dq = varargin{1};
            ddq = varargin{2};
            dibujaSliderCrank(q,dq,ddq)
        end
    case 3 % Biela Manivela
        if (nargin==2)
            dibujaCoriolis(q)
        end
        if (nargin==3)
            dq = varargin{1};
            dibujaCoriolis(q,dq)
        end
        if (nargin>=4)
            dq = varargin{1};
            ddq = varargin{2};
            dibujaCoriolis(q,dq,ddq)
        end
end
%-----------------------------------------------------
function [ ] = dibujaCuatroBarras( q, varargin )
% Dibuja un mecanismoe de cuatro barras, con las dos manivelas como discos,
%  para las practicas de Teoria de Mecanismos.
% * "q" debe ser un vector de tamaño 4 con las coordenadas naturales de los
%   puntos 1 y 2. Es decir: q=[x1 y1 x2 y2]
%
% * Se asume que el origen de la primera manivela estï¿½ en (0,0).
%
% * IMPORTANTE: Las coordenadas de la tierra de la manivela derecha (xB,yB)
%               deben cambiarse mï¿½s abajo, segun corresponda.
%
% * Si se pasan los vectores (opcionales) de dq y ddq, se dibujaran las
%   flechas de velocidad y aceleracion en cada punto.
%
%  Ejemplo: dibujaCuatroBarras([-0.3 0.2  0.5 0.1 ])
%
% (C) 2013 Jose Luis Blanco, Jose Luis Torres, Antonio Gimenez Fernandez
%  Departamento de Ingenieria, Universidad de Almeria. Licencia GNU GPL 3.
%

%global mpSt; % La estructura de mechplot, para reutilizarla entre llamadas.
%global hAx; % Axis handle, para detectar si se cierra la figura
%global xA yA xB yB
xA = q(7);
yA = q(8);
xB = q(9);
yB = q(10);
% % Posicion del punto de tierra de la manivela de la izquierda (xA,yA)
% xA = 0;
% yA = 0;
%
% % Posicion del punto de tierra de la manivela de la derecha (xB,yB)
% xB = 285.0; % en milimetros
% yB = 0; % en milimetros

% % Radios externos (fijos) de los discos:
% Radio1 = 137.5;
% Radio2 = 137.5;
% Expandir el vector q para que dibuje los discos con radios externos
% constantes, independientemente del lugar de enganche de la biela:
% q = [x1 y1 x2  y2  ...  x1e y1e  x2e y2e ]
%       1  2  3  4   ...   10  11   12  13
th1=atan2(q(2)-yA,q(1)-xA);
th2=atan2(q(4)-yB,q(3)-xB);
Radio1 = 1.1; Radio2 = Radio1;
q(10:11)=[xA,yA]+[cos(th1),sin(th1)]*Radio1;
q(12:13)=[xB,yB]+[cos(th2),sin(th2)]*Radio2;

% Formar vector de coordenadas "fijas":
q_fixed=[xA yA xB yB];

new_hAx = gca();

if (isprop(handle(gca),'hMechPlot'))
    mpSt = get(new_hAx,'hMechPlot');
    % Reutilizar el objeto y solo actualizar las coordenadas no fijas:
    mpSt.keep_axis_limits = 1;
    mpSt.skip_drawnow=1;
    %mpSt.save_images = 0; mpSt.bg_color=[1 1 1]; mpSt.save_image_filenames = 'mechplot_%03d.png';
    mpSt.plot(q);
else
    % Crear el objeto dibujo del mecanismo con "mechplot":
    % addpath('./libmechplot');
    mpSt= creaMP4Barras(q,q_fixed);
    
    % Embed the MechPlot handle into the axis:
    if (exist('schema.prop','class')>0)
        schema.prop(handle(gca), 'hMechPlot','MATLAB array');
        set(gca,'hMechPlot',mpSt);
    end
    
end

if (nargin>=2)
    dq = varargin{1};
    if (~isempty(dq))
        % Dibuja flecha de velocidad:
        arrow(reshape(q(1:4),2,2)',reshape(q(1:4)+dq(1:4)*0.5,2,2)', 'EdgeColor','b','FaceColor','b','Width',3);
    end
end

if (nargin>=3)
    ddq = varargin{2};
    if (~isempty(ddq))
        % Dibuja flecha de aceleracion:
        arrow(reshape(q(1:4),2,2)',reshape(q(1:4)+ddq(1:4)*0.5,2,2)', 'EdgeColor','r','FaceColor','r','Width',2);
    end
end

%drawnow expose update;

%-----------------------------------------------------------------------
function [s] = creaMP4Barras(q, q_fixed)
s=mpMechanism();  % Create new, empty mechanism object

%----------- - - -
% LINK #2
b = mpLink('id',2,...
    'z_order',1 ... % props de barra 2
    );
b.points(end+1) = mpPoint('is_fixed',1, 'fixed_x_idx',1, 'fixed_y_idx',2);
b.points(end+1) = mpPoint('is_fixed',0, 'x_idx',1, 'y_idx',2);
s.add(b);

% LINK #3
b = mpLink('id',3,...
    'z_order',1 ... % props de barra 3
    );
b.points(end+1) = mpPoint('is_fixed',0, 'x_idx',1, 'y_idx',2);
b.points(end+1) = mpPoint('is_fixed',0, 'x_idx',3, 'y_idx',4);
s.add(b);

%----------------
b = mpLink('id',4,...
    'z_order',1 ... % props de barra 4
    );
b.points(end+1) = mpPoint('is_fixed',0, 'x_idx',3, 'y_idx',4);
%b.points(end+1) = mpPoint('is_fixed',1, 'x_idx',3, 'y_idx',4);
b.points(end+1) = mpPoint('is_fixed',1, 'fixed_x_idx',3, 'fixed_y_idx',4);
s.add(b);
%-------
% Ground points #1 and #2:
s.add( mpPinnedGround(1,2,'drawGroundHatch',0) );
s.add( mpPinnedGround(3,4,'drawGroundHatch',0) );

% Set fixed points:
s.q_fixed = q_fixed;

s.plot(q);


%------------------------------------------------------------------------
function [ ] = dibujaSliderCrank( q_qfixed, varargin )
%DIBUJABIELACORREDERA Dibuja el mecanismoe de Biela corredera para el ejemplo
%     de prácticas de Teoría de Mecanismos.
% * "q" debe ser un vector de tamaño 4 con las coordenadas naturales de los
%   puntos 1 y 2. Es decir: q=[x1 y1 x2 y2]
% * Se asume que el origen de la manivela está en (0,0).
% * Se asume que la corredera es horizontal, en y=-1
% * Si se pasan los vectores (opcionales) de dq y ddq, se dibujaran las
%   flechas de velocidad y aceleracion en cada punto.

%  Ejemplo: dibujaBielaCorredera([1 1 5 -1])
%
% (C) 2013 Jose Luis Blanco, Jose Luis Torres, Antonio Gimenez Fernandez
%  Departamento de Ingenieria, Universidad de Almeria. Licencia GNU GPL 3.
%

% Formar vector de coordenadas "fijas":
q_fixed=q_qfixed(7:12);
% Formar vector de coordenadas "moviles":
q=q_qfixed(1:4); % Ignorar la coordenada relativa, solo dibujar usando las coordenadas Cartesianas.
% %L2=25; L3=30; %
% L2=sqrt((q_qfixed(1)-q_qfixed(7))^2+(q_qfixed(2)-q_qfixed(8))^2);
% L3=sqrt((q_qfixed(3)-q_qfixed(1))^2+(q_qfixed(4)-q_qfixed(2))^2);

if (isprop(handle(gca),'hMechPlot'))
    mpSt = get(gca,'hMechPlot');
    % Reutilizar el objeto y solo actualizar las coordenadas no fijas:
    mpSt.keep_axis_limits = 1;
    mpSt.skip_drawnow=1;
    mpSt.plot(q);
else
    % Crear el objeto dibujo del mecanismo con "mechplot":
    mpSt= creaMPbielaManivela(q,q_fixed);
    
    % Embed the MechPlot handle into the axis:
    if (exist('schema.prop','class')>0)
        schema.prop(handle(gca), 'hMechPlot','MATLAB array');
        set(gca,'hMechPlot',mpSt);
    end
end

if (nargin>=2)
    dq = varargin{1};
    if (~isempty(dq))
        % Dibuja flecha de velocidad:
        arrow(reshape(q(1:4),2,2)',reshape(q(1:4)+dq(1:4)*0.5,2,2)', 'EdgeColor','b','FaceColor','b','Width',3);
    end
end

if (nargin>=3)
    ddq = varargin{2};
    if (~isempty(ddq))
        % Dibuja flecha de aceleracion:
        arrow(reshape(q(1:4),2,2)',reshape(q(1:4)+ddq(1:4)*0.5,2,2)', 'EdgeColor','r','FaceColor','r','Width',2);
    end
end
%drawnow expose update;

function [s] = creaMPbielaManivela(q, q_fixed)
s=mpMechanism();  % Create new, empty mechanism object

% Dummy fixed bar, guide of the slider block.
%    b = mpLink('r',[0 0],extra_params{4}{:});
b = mpLink('r',[0 0]);
b.points(end+1) = mpPoint('is_fixed',1, 'fixed_x_idx',3, 'fixed_y_idx',4);
b.points(end+1) = mpPoint('is_fixed',1, 'fixed_x_idx',5, 'fixed_y_idx',6);
s.add(b);

% Slider block over fixed guide
%    b = mpSliderBlock('id',4,extra_params{3}{:});
b = mpSliderBlock('id',4);
b.joint_point = mpPoint('is_fixed',0, 'x_idx',3, 'y_idx',4);
b.guide_points(1) = mpPoint('is_fixed',1, 'fixed_x_idx',3, 'fixed_y_idx',4);
b.guide_points(2) = mpPoint('is_fixed',1, 'fixed_x_idx',5, 'fixed_y_idx',6);
s.add(b);


% Link #1 is the "ground/fixed element"

% LINK #2
%b = mpLink('id',2,extra_params{1}{:});
b = mpLink('id',2);
b.points(end+1) = mpPoint('is_fixed',1, 'fixed_x_idx',1, 'fixed_y_idx',2);
b.points(end+1) = mpPoint('is_fixed',0, 'x_idx',1, 'y_idx',2);
s.add(b);

% LINK #3
%b = mpLink('id',3,extra_params{2}{:});
b = mpLink('id',3);
b.points(end+1) = mpPoint('is_fixed',0, 'x_idx',1, 'y_idx',2);
b.points(end+1) = mpPoint('is_fixed',0, 'x_idx',3, 'y_idx',4);
s.add(b);


% Ground point #1
%s.add( mpPinnedGround(1,2,extra_params{5}{:}) );
s.add( mpPinnedGround(1,2));

% Ground points of the fixed guide bar:
guide_ang = atan2(q_fixed(6)-q_fixed(4),q_fixed(5)-q_fixed(3));
%     s.add( mpFixedGround(3,4,'orientation',0+guide_ang, extra_params{6}{:} ));
%     s.add( mpFixedGround(5,6,'orientation',pi+guide_ang, extra_params{7}{:} ));
s.add( mpFixedGround(3,4,'orientation',0+guide_ang));
s.add( mpFixedGround(5,6,'orientation',pi+guide_ang));
% Set fixed points:
s.q_fixed = q_fixed;
s.plot(q);


%-------------------------------------------------------------------------
function [ ] = dibujaCoriolis( q_qfixed, varargin )
%DIBUJABIELACORREDERA Dibuja el mecanismoe de Biela corredera para el ejemplo
%     de prácticas de Teoría de Mecanismos.
% * "q" debe ser un vector de tamaño 4 con las coordenadas naturales de los
%   puntos 1 y 2. Es decir: q=[x1 y1 x2 y2]
% * Se asume que el origen de la manivela está en (0,0).
% * Se asume que la corredera es horizontal, en y=-1
% * Si se pasan los vectores (opcionales) de dq y ddq, se dibujaran las
%   flechas de velocidad y aceleracion en cada punto.

%  Ejemplo: dibujaBielaCorredera([1 1 5 -1])
%
% (C) 2013 Jose Luis Blanco, Jose Luis Torres, Antonio Gimenez Fernandez
%  Departamento de Ingenieria, Universidad de Almeria. Licencia GNU GPL 3.
%


% Formar vector de coordenadas "fijas":
%q_fixed=[q_qfixed(8:11); q_qfixed(12:13)];
%q_fixed=[q_qfixed(8:9); q_qfixed(10:11); q_qfixed(12:13)];
q_fixed=[q_qfixed(7:8); q_qfixed(9:10); q_qfixed(1:2)];
% Formar vector de coordenadas "moviles":
q=q_qfixed(1:4); % Ignorar la coordenada relativa, solo dibujar usando las coordenadas Cartesianas.

% L2=sqrt((q_qfixed(1)-q_qfixed(8))^2+(q_qfixed(2)-q_qfixed(9))^2);
% L3=sqrt((q_qfixed(3)-q_qfixed(10))^2+(q_qfixed(4)-q_qfixed(11))^2);

if (isprop(handle(gca),'hMechPlot'))
    mpSt = get(gca,'hMechPlot');
    % Reutilizar el objeto y solo actualizar las coordenadas no fijas:
    mpSt.keep_axis_limits = 1;
    mpSt.skip_drawnow=1;
    mpSt.plot(q);
else
    % Crear el objeto dibujo del mecanismo con "mechplot":
    mpSt= creaMPcoriolis(q,q_fixed);
    
    % Embed the MechPlot handle into the axis:
    if (exist('schema.prop','class')>0)
        schema.prop(handle(gca), 'hMechPlot','MATLAB array');
        set(gca,'hMechPlot',mpSt);
    end
end

if (nargin>=2)
    dq = varargin{1};
    if (~isempty(dq))
        % Dibuja flecha de velocidad:
        arrow(reshape(q(1:4),2,2)',reshape(q(1:4)+dq(1:4)*0.5,2,2)', 'EdgeColor','b','FaceColor','b','Width',3);
    end
end

if (nargin>=3)
    ddq = varargin{2};
    if (~isempty(ddq))
        % Dibuja flecha de aceleracion:
        arrow(reshape(q(1:4),2,2)',reshape(q(1:4)+ddq(1:4)*0.5,2,2)', 'EdgeColor','r','FaceColor','r','Width',2);
    end
end

%drawnow expose update;


function [s] = creaMPcoriolis(q, q_fixed)
s=mpMechanism();  % Create new, empty mechanism object

% Slider block over fixed guide
b = mpSliderBlock('id',4);
b.joint_point = mpPoint('is_fixed',0, 'x_idx',1, 'y_idx',2);
b.guide_points(1) = mpPoint('is_fixed',1, 'fixed_x_idx',3, 'fixed_y_idx',4);
%b.guide_points(2) = mpPoint('is_fixed',1, 'fixed_x_idx',5, 'fixed_y_idx',6);
b.guide_points(2) = mpPoint('is_fixed',0, 'x_idx',1, 'y_idx',2);
s.add(b);


% Link #1 is the "ground/fixed element"

% LINK #2
b = mpLink('id',2);
b.points(end+1) = mpPoint('is_fixed',1, 'fixed_x_idx',1, 'fixed_y_idx',2);
b.points(end+1) = mpPoint('is_fixed',0, 'x_idx',1, 'y_idx',2);
s.add(b);

% LINK #3
b = mpLink('id',3);
b.points(end+1) = mpPoint('is_fixed',0, 'x_idx',3, 'y_idx',4);
b.points(end+1) = mpPoint('is_fixed',1, 'fixed_x_idx',3, 'fixed_y_idx',4);
s.add(b);

% Ground point #1
s.add( mpPinnedGround(1,2));

% Ground points of the second bar:
s.add( mpFixedGround(3,4,'orientation',0));

% Set fixed points:
s.q_fixed = q_fixed;
s.plot(q);

function [] = resultados(handles,mbs,q, varargin )
rotaciones = zeros(3,3);
coordenadas = zeros(5,2);
coordenadas(1,:) = [mbs.xA, mbs.yA];

switch mbs.mechanism_type
    case 1 % Cuatro barras
        % V i r t u a l S e n s o r s
        migiroManivela = mbeSensorGyroscope(...
            mbs.installed_sensors{1}.pt_is_fixed,...
            mbs.installed_sensors{1}.pt1_idxs,...
            mbs.installed_sensors{1}.pt2_idxs, 0);
        migiroAcoplador = mbeSensorGyroscope(...
            mbs.installed_sensors{2}.pt_is_fixed,...
            mbs.installed_sensors{2}.pt1_idxs,...
            mbs.installed_sensors{2}.pt2_idxs, 0);
        migiroBalancin = mbeSensorGyroscope(...
            mbs.installed_sensors{3}.pt_is_fixed,...
            mbs.installed_sensors{3}.pt1_idxs,...
            mbs.installed_sensors{3}.pt2_idxs, 0);
        miangleManivela = mbeSensorAngle(...
            mbs.installed_sensors{4}.pt_is_fixed,...
            mbs.installed_sensors{4}.pt1_idxs,...
            mbs.installed_sensors{4}.pt2_idxs, 0);
        mianguloAcoplador = mbeSensorAngle(...
            mbs.installed_sensors{5}.pt_is_fixed,...
            mbs.installed_sensors{5}.pt1_idxs,...
            mbs.installed_sensors{5}.pt2_idxs, 0);
        miangleBalancin = mbeSensorAngle(...
            mbs.installed_sensors{6}.pt_is_fixed,...
            mbs.installed_sensors{6}.pt1_idxs,...
            mbs.installed_sensors{6}.pt2_idxs, 0);
        
        % Posicion
        set(handles.editBeta,'String',q(6))
        z2= q(5);
        z3= mianguloAcoplador.sensor_simulate(mbs,q);
        z4=q(end);
        
        rotaciones(1,1) = rad2deg(z2);
        rotaciones(1,2) = rad2deg(z3);
        rotaciones(1,3) = rad2deg(z4);
        
        coordenadas(2,:) = [q(1), q(2)];
        coordenadas(3,:) = [q(3), q(4)];
        coordenadas(4,:) = [mbs.xB, mbs.yB];
        
        % Velocidad
        if (nargin>=4)
            qp = varargin{1};
            o2= qp(5);
            o3= migiroAcoplador.sensor_simulate(mbs,q,qp);
            o4=qp(end);
            rotaciones(2,1) = o2;
            rotaciones(2,2) = o3;
            rotaciones(2,3) = o4;
        end
        
        % Aceleracion
        if (nargin>=5)
            qpp = varargin{2};
            a2 = qpp(5);
            a4 = qpp(end);
            
            rotaciones(3,1) = a2;
            rotaciones(3,2) = 0; % TODO: en realidad no tiene por que
            rotaciones(3,3) = a4;
        end
        
    case 2 % Biela manivela
        % V i r t u a l S e n s o r s
        migiroManivela = mbeSensorGyroscope(...
            mbs.installed_sensors{1}.pt_is_fixed,...
            mbs.installed_sensors{1}.pt1_idxs,...
            mbs.installed_sensors{1}.pt2_idxs, 0);
        migiroAcoplador = mbeSensorGyroscope(...
            mbs.installed_sensors{2}.pt_is_fixed,...
            mbs.installed_sensors{2}.pt1_idxs,...
            mbs.installed_sensors{2}.pt2_idxs, 0);
        mianguloAcoplador = mbeSensorAngle(...
            mbs.installed_sensors{3}.pt_is_fixed,...
            mbs.installed_sensors{3}.pt1_idxs,...
            mbs.installed_sensors{3}.pt2_idxs, 0);
        
        
        % Posicion
        z2 = q(5);
        z3= mianguloAcoplador.sensor_simulate(mbs,q);
        set(handles.editS,'String',q(6))
        rotaciones(1,1) = rad2deg(z2);
        rotaciones(1,2) = rad2deg(z3);
        coordenadas(2,:) = [q(1), q(2)];
        coordenadas(3,:) = [q(3), q(4)];
        coordenadas(4,:) = [mbs.xB, mbs.yB];
        coordenadas(5,:) = [mbs.xC, mbs.yC];
        % Velocidad
        if (nargin>=4)
            qp = varargin{1};
            %o2= migiroManivela.sensor_simulate(mbs,q,qp);
            o2= qp(5);
            o3= migiroAcoplador.sensor_simulate(mbs,q,qp);
            
            rotaciones(2,1) = o2;
            rotaciones(2,2) = o3;
        end
        
        % Aceleracion
        if (nargin>=5)
            qpp = varargin{2};
            a2=qpp(5);
            
            rotaciones(3,1) = a2;
        end
        
    case 3 % Coriolis
        % Posicion
        z2=q(5);
        z3=q(end);
        set(handles.editBeta,'String',q(6))
        rotaciones(1,1) = rad2deg(z2);
        rotaciones(1,2) = rad2deg(z3);
        coordenadas(2,:) = [q(1), q(2)];
        coordenadas(3,:) = [q(3), q(4)];
        coordenadas(4,:) = [mbs.xB, mbs.yB];
        % Velocidad
        if (nargin>=4)
            qp = varargin{1};
            o2=qp(5);
            o3=qp(end);
            
            rotaciones(2,1) = o2;
            rotaciones(2,2) = o3;
        end
        
        % Aceleracion
        if (nargin>=5)
            qpp = varargin{2};
            a2=qpp(5);
            a3=qpp(end);
            
            rotaciones(3,1) = a2;
            rotaciones(3,2) = a3;
        end
        
        
        
end
% Posicion
pause(0.0000001)
set(handles.tablaRot,'Data',rotaciones);
set(handles.tablaCoord,'Data',coordenadas);
set(handles.editPosGDL,'String',q(5))

% Velocidad
if (nargin>=4)
    set(handles.editVelGDL,'String',qp(5))
    set(handles.editVx1,'String',qp(1));
    set(handles.editVy1,'String',qp(2));
    set(handles.editModV1,'String',sqrt(qp(1)^2+qp(2)^2));
    set(handles.editVx2,'String',qp(3));
    set(handles.editVy2,'String',qp(4));
    set(handles.editModV2,'String',sqrt(qp(3)^2+qp(4)^2));
    set(handles.editDirV1,'String',rad2deg(atan((qp(2)/qp(1)))));
    set(handles.editDirV2,'String',rad2deg(atan((qp(4)/qp(3)))));
end

% Aceleracion
if (nargin>=5)
    qpp = varargin{2};
    set(handles.editAx1,'String',qpp(1));
    set(handles.editAy1,'String',qpp(2));
    set(handles.editModA1,'String',sqrt(qpp(1)^2+qpp(2)^2));
    set(handles.editAx2,'String',qpp(3));
    set(handles.editAy2,'String',qpp(4));
    set(handles.editModA2,'String',sqrt(qpp(3)^2+qpp(4)^2));
    set(handles.editDirA1,'String',rad2deg(atan((qpp(2)/qpp(1)))));
    set(handles.editDirA2,'String',rad2deg(atan((qpp(4)/qpp(3)))));
end



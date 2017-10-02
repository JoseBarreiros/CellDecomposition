
%Exact Cell Decomposition [ECD]%
%Implementation of Path Planning Algorith using ECD%
%The algorithm identify the cells and C-objects from an XML file that include a C-workspace (.cmap), then find
%the distance between the centroids of each cell and calculate the shortest
%path.  The actual trajectory for the robot is a set of lines from the midpoint of
%every cell included in the path.
%Created by Jose Barreiros, Oct. 2017. 
%PhD student.
%Cornell University. 
%The XML script and GUI structure are based on the work of Ahmad Abbadi, Brno University of Technology (VUT), Czech Republic


function varargout = Cell_Decomposition_v2(varargin)

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Cell_Decomposition_v2_OpeningFcn, ...
    'gui_OutputFcn',  @Cell_Decomposition_v2_OutputFcn, ...
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
end

function Cell_Decomposition_v2_OpeningFcn(hObject, eventdata, handles, varargin)

handles.drawingFigHandle=handles.axes1;


handles.obstacles=[];
handles.cells=[];
handles.pois=[];
handles.currentobstacleIndex=[];
handles.file.FileName=[];
handles.file.PathName=[];
handles.changesaved=1;
handles.axesAttribute.xlim=[0 10];
handles.axesAttribute.ylim=[0 10];

adjacent=[];
w_adj=[];
coord=[];
path_vertex=[];

handles.SPosition.center=[];
handles.SPosition.radius=[];
handles.SPosition.h=[];
handles.SPosition.type=[];

handles.GPosition.center=[];
handles.GPosition.radius=[];
handles.GPosition.h=[];
handles.GPosition.type=[];

handles.cellDecom.graph=[];
handles.cellDecom.cells=[];


set(handles.drawingFigHandle,'xlim',handles.axesAttribute.xlim,'ylim',handles.axesAttribute.ylim);
hold on;

% Choose default command line output for planner_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes planner_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

end

function LoadCSpace_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to LoadCSpace_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[FileName,PathName,FilterIndex] = uigetfile('*.cmap','Open File');
if (FileName~=0)
    
    handles.file.FileName=FileName;
    handles.file.PathName=PathName;
    if size(handles.obstacles,1) >0
        delete(handles.obstacles(:).linehandles);
        handles.obstacles=[];
        handles.currentobstacleIndex=[];
        
        
    end
    
    
    ObstacleXML=xmlread(strcat(PathName,FileName));
    scpaceNode=ObstacleXML.getDocumentElement;
    dimension= str2double(scpaceNode.getAttribute('dim'));
    if dimension~=2
        h=warndlg('Can not open dimention oher than 2');
        uiwait(h);
    else
        
        % retrieve obstacles
        obstacles=scpaceNode.getElementsByTagName('obstacle');
        cells=scpaceNode.getElementsByTagName('cell');
        pois=scpaceNode.getElementsByTagName('poi');
        obsLength=obstacles.getLength - 1;
        cellLength=cells.getLength - 1;
        poiLength=pois.getLength - 1;
        for i= 0 : obsLength
            obs=obstacles.item(i);
            newobstacle=[];
            
            newobstacle.type=char(obs.getAttribute('type'));
            
            vertices=obs.getElementsByTagName('vertex');
            verLength=vertices.getLength-1;
            for j=0:verLength
                newobstacle.vertex(j+1,1)=str2double(vertices.item(j).getAttribute('x'));
                newobstacle.vertex(j+1,2)=str2double(vertices.item(j).getAttribute('y'));
            end
            
            
            newobstacle.ID=i+1;
            handles.obstacles=[handles.obstacles;newobstacle];
            
        end
           % retrieve cells
        for i= 0 : cellLength
            cel=cells.item(i);
            newcell=[];
            
            newcell.type=char(cel.getAttribute('type'));
            
            vertices=cel.getElementsByTagName('vertex');
            verLength=vertices.getLength-1;
            for j=0:verLength
                newcell.vertex(j+1,1)=str2double(vertices.item(j).getAttribute('x'));
                newcell.vertex(j+1,2)=str2double(vertices.item(j).getAttribute('y'));
            end
            
            
            newcell.ID=i+1;
            handles.cells=[handles.cells;newcell];
            
        end
        
        % retrieve points of interest POI: start and end condition
        for i= 0 : poiLength
            poi=pois.item(i);
            newpoi=[];
            
            newpoi.type=char(poi.getAttribute('type'));
            
            vertices=poi.getElementsByTagName('point');
            verLength=vertices.getLength-1;
            for j=0:verLength
                newpoi.vertex(j+1,1)=str2double(vertices.item(j).getAttribute('x'));
                newpoi.vertex(j+1,2)=str2double(vertices.item(j).getAttribute('y'));
            end
            
            
            newpoi.ID=i+1;
            handles.pois=[handles.pois;newpoi];
            
        end
        
        
        % retrieve Axes sitting
        AxesNode=scpaceNode.getElementsByTagName('Axes');
        handles.axesAttribute.xlim = str2num(AxesNode.item(0).getAttribute('xlim'));
        handles.axesAttribute.ylim = str2num(AxesNode.item(0).getAttribute('xlim'));
        set(handles.drawingFigHandle,'xlim',handles.axesAttribute.xlim,'ylim',handles.axesAttribute.ylim);
        hold on;
        
 
        %draw obstacle
        for i=1:size(handles.obstacles,1)
            handles.obstacles(i).linehandles=fill([handles.obstacles(i).vertex(:,1);handles.obstacles(i).vertex(1,1)],[handles.obstacles(i).vertex(:,2);handles.obstacles(i).vertex(1,2)],'b');
            alpha(handles.obstacles(i).linehandles,0.3);% color transparency
            set(handles.obstacles(i).linehandles,'EdgeColor','none');% edge color of obstacle
            
        end
        
        %draw cells
        for i=1:size(handles.cells,1)
            handles.cells(i).linehandles=fill([handles.cells(i).vertex(:,1);handles.cells(i).vertex(1,1)],[handles.cells(i).vertex(:,2);handles.cells(i).vertex(1,2)],'g');
            alpha(handles.cells(i).linehandles,0.2);% color transparency
            set(handles.cells(i).linehandles,'EdgeColor','r');% edge color of obstacle
            [ geom, iner, cpmo ] = polygeom([handles.cells(i).vertex(:,1);handles.cells(i).vertex(1,1)],[handles.cells(i).vertex(:,2);handles.cells(i).vertex(1,2)]);
            area = geom(1);
            handles.cells(i).centroid(1) = geom(2);
            handles.cells(i).centroid(2) = geom(3);
            x_cen=handles.cells(i).centroid(1)
            y_cen=handles.cells(i).centroid(2)
            txt=int2str(i);
           % text(x_cen,y_cen,txt);
            coord(i,:)=[x_cen y_cen]; %create coordinates for connectivity graph
            handles.coord(i,:)=[x_cen y_cen];
        end
        tic;
        
    end
end

hObject.UserData = handles;

end

function Path_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to LoadCSpace_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
h = findobj('Tag','LoadCSpace_pushbutton');
handles = h.UserData
coord = handles.coord

        
        %test for adjacency
        adjacent=zeros(size(handles.cells,1));
        w_adj=zeros(size(handles.cells,1));
        x=zeros(size(handles.cells,1),1)
        y=zeros(size(handles.cells,1),1)
        v_count=0;
        for i=1:size(handles.cells,1)
            for j=1:size(handles.cells,1)
                v_count=0;
                for k=1:subsref(size(handles.cells(i).vertex),struct('type','()','subs',{{1,1}}))
                    for m=1:subsref(size(handles.cells(j).vertex),struct('type','()','subs',{{1,1}}))
                        if isequal(handles.cells(i).vertex(k,:),handles.cells(j).vertex(m,:))==1
                            v_count=v_count+1;  %count common vertices
                            if v_count>1  %if cells share at least 2 common vertices then the cells are adjacent
                                adjacent(i,j)=1;  
                            end
                        end
                    end
                end
            end
        end
        
        adjacent;
        
        %plot adjacent matrix
        gplot(adjacent,coord,'-*')
        %build connectivity graph and find shortest path
        for i=1:size(handles.cells,1)
                    xi=handles.cells(i).centroid(1)
                    x(i)=xi
                    yi=handles.cells(i).centroid(2)
                    y(i)=yi
            for j=i+1:size(handles.cells,1)
                    xj=handles.cells(j).centroid(1)
                    yj=handles.cells(j).centroid(2)
                if adjacent(i,j)==1

                    distance=pdist([xi,yi;xj,yj],'euclidean')
                    w_adj(i,j)=distance;  %build a weighted adjacency matrix
                    mid_x=(xi+xj)/2
                    mid_y=(yi+yj)/2
                    txt=num2str(distance,4);
                    text(mid_x,mid_y,txt);
                end
            end
        end
        w_adj 
        G=graph(w_adj,'upper')
        plot(G, 'XData', x, 'YData', y,'LineWidth',2, 'EdgeColor',[0,0.7,0.9])
        P = shortestpath(G,3,8)
 
              %draw POIs
        for i=1:size(handles.pois,1)
            %handles.pois(i).linehandles=fill([handles.pois(i).vertex(:,1);handles.pois(i).vertex(1,1)],[handles.pois(i).vertex(:,2);handles.pois(i).vertex(1,2)],'c');
           plot(handles.pois(i).vertex(1,1),handles.pois(i).vertex(1,2),'ks')
           txt=handles.pois(i).type
           tex=text(handles.pois(i).vertex(1,1),handles.pois(i).vertex(1,2),txt)
           tex.FontSize=11
           tex.FontWeight = 'bold'
%            alpha(handles.pois(i).linehandles,0.4);% color transparency
           % set(handles.pois(i).linehandles,'EdgeColor','k');% edge color of obstacle
            
        end
        
        %find the matrix of the shortest path line using the midpoints of the edges
        for i=1:size(P,2)
            i
            for j=1:subsref(size(handles.cells(P(i)).vertex),struct('type','()','subs',{{1,1}}))
                j
                if j==subsref(size(handles.cells(P(i)).vertex),struct('type','()','subs',{{1,1}}))
                    k=1
                else
                    k=j+1
                end
               lx=[handles.cells(P(i)).centroid(1) handles.cells(P(i)+1).centroid(1) handles.cells(P(i)).vertex(j,1) handles.cells(P(i)).vertex(k,1)]
               ly=[handles.cells(P(i)).centroid(2) handles.cells(P(i)+1).centroid(2) handles.cells(P(i)).vertex(j,2) handles.cells(P(i)).vertex(k,2)]
               dt1=det([1,1,1;lx(1),lx(2),lx(3);ly(1),ly(2),ly(3)])*det([1,1,1;lx(1),lx(2),lx(4);ly(1),ly(2),ly(4)])
                dt2=det([1,1,1;lx(1),lx(3),lx(4);ly(1),ly(3),ly(4)])*det([1,1,1;lx(2),lx(3),lx(4);ly(2),ly(3),ly(4)])

                if(dt1<=0 & dt2<=0)
                    
                    path_vertex(i,:)=[(lx(3)+lx(4))/2 (ly(3)+ly(4))/2]
                end   
            end
        end
        
        path_vertex
        path_vertex=cat(1,handles.pois(1).vertex,path_vertex)
        path_vertex(i+1,:)=handles.pois(2).vertex
        %plot minimum path from start to end point
        plot(path_vertex(:,1),path_vertex(:,2),'k','LineWidth',3)

        toc
        % new load; no change
        handles.changesaved=1;
        % update handles data
        guidata(hObject, handles);
        
    end


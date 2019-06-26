function neighbor_set = create_neighbor_set(ShapeModel)


% clear, clc, close all, run ../startup.m  % refresh

% Add Required Packages to PATH
% addpath(genpath(strcat(ROOT_PATH,'/small_body_dynamics/EROS 433')))
% addpath(strcat(ROOT_PATH,'/small_body_dynamics'))
% addpath(genpath(strcat(ROOT_PATH,'/utilities'))) % Add all utilities
% addpath(strcat(ROOT_PATH,'/visualization'))
% 
% % Spherical harmonics model. Format is specified in the MATLAB function 'gravitysphericalharmonic.m'.
% SphericalModel = {'EROS 433/Gravity_models/n15acoeff.tab', @readErosGravityModel};
% 
% % Load physical parameters of asteroid as a struct
% ErosParameters = get_Eros_body_parameters(SphericalModel{1}); % input is optional
% 
% % Name and path of shapefile. For now, only used for plotting.
% shapefilename = 'EROS 433/MSI_optical_plate_models/eros022540.tab';
% 
% ErosShapeModel = get_shape_model(shapefilename) ; % Faces and verticies of Eros model

%
G = ShapeModel;

V = G.Vertices;
E = G.Faces;

% figure()
% hold on

% find and plot neighbors of v1:
n_vertices = size(V,1);
n_faces    = size(E,1);
vertex_set = 1:n_vertices;
N = cell(n_vertices, 1); % neighbor_set{i} returns neighbors of ith vertex
neighbor_set = cell(n_vertices, 1); 
N = cell(n_vertices, 1); 
for j = vertex_set
    for i = 1:n_faces
        
        if E(i,1)==j %|| E(i,2)==1 || E(i,3)==1
            if sum(N{j}==E(i,2))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,2)];
            end
            if sum(N{j}==E(i,3))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,3)];
            end
        end
        
        if E(i,2)==j %|| E(i,2)==1 || E(i,3)==1
            if sum(N{j}==E(i,1))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,1)];
            end
            if sum(N{j}==E(i,3))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,3)];
            end
        end
        
        if E(i,3)==j %|| E(i,2)==1 || E(i,3)==1
            if sum(N{j}==E(i,1))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,1)];
            end
            if sum(N{j}==E(i,2))==0 % if not aleady in neighborset add
                N{j} = [N{j},E(i,2)];
            end
        end
        
    end
    
    neighbor_set{j} = sort(N{j}); 

end


end


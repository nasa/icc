%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2019 by California Institute of Technology.  ALL RIGHTS RESERVED. %
% United  States  Government  sponsorship  acknowledged.   Any commercial use %
% must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the %
% California Institute of Technology.                                         %
%                                                                             %
% This software may be subject to  U.S. export control laws  and regulations. %
% By accepting this document,  the user agrees to comply  with all applicable %
% U.S. export laws and regulations.  User  has the responsibility  to  obtain %
% export  licenses,  or  other  export  authority  as may be required  before %
% exporting  such  information  to  foreign  countries or providing access to %
% foreign persons.                                                            %
%                                                                             %
% This  software  is a copy  and  may not be current.  The latest  version is %
% maintained by and may be obtained from the Mobility  and  Robotics  Sytstem %
% Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches %
% are welcome and should be sent to the software's maintainer.                %
%                                                                             %
% Author: Saptarshi Bandyopadhyay (saptarshi.bandyopadhyay@jpl.nasa.gov)      %
%                                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function obj = compute_normals_shape_model(obj)

disp('Start Normal calculation')

vertices = obj.BodyModel.shape.vertices;
faces = obj.BodyModel.shape.faces; 

faces_normals = 0*faces;
faces_normals_confidence = zeros(size(faces,1),1);

iteration_number = 0;
disp(['Iteration :',num2str(iteration_number)])

for i=1:1:size(faces,1)
    %     disp(['Iteration :',num2str(iteration_number),', Point :',num2str(i)])
    
    this_pts = vertices(faces(i,:),:);
    
    this_face_normal = cross(this_pts(2,:)-this_pts(1,:), this_pts(3,:)-this_pts(1,:));
    this_face_normal = this_face_normal/norm(this_face_normal);
    
    this_angle = acosd(dot(this_face_normal, this_pts(1,:)/norm(this_pts(1,:))));
    
    if this_angle >= 130
        faces_normals_confidence(i,1) = 1;
        faces_normals(i,:) = -this_face_normal;
    elseif this_angle <=50
        faces_normals_confidence(i,1) = 1;
        faces_normals(i,:) = this_face_normal;
    else
        faces_normals_confidence(i,1) = 0.5;
        faces_normals(i,:) = this_face_normal;
    end
    
end

while sum(faces_normals_confidence==0.5) > 0
    iteration_number = iteration_number + 1;
    disp(['Iteration :',num2str(iteration_number)])
    
    for i=1:1:size(faces,1)
        
        if faces_normals_confidence(i,1) == 0.5
            %             disp(['Iteration :',num2str(iteration_number),', Point :',num2str(i)])
            
            this_face_pts = faces(i,:);
            
            all_faces_pts1 = [find(faces(:,1)==this_face_pts(1)); find(faces(:,2)==this_face_pts(1)); find(faces(:,3)==this_face_pts(1))];
            all_faces_pts2 = [find(faces(:,1)==this_face_pts(2)); find(faces(:,2)==this_face_pts(2)); find(faces(:,3)==this_face_pts(2))];
            all_faces_pts3 = [find(faces(:,1)==this_face_pts(3)); find(faces(:,2)==this_face_pts(3)); find(faces(:,3)==this_face_pts(3))];
            
            all_faces_pts1_high_confidence = all_faces_pts1(find(faces_normals_confidence(all_faces_pts1)==1));
            all_faces_pts2_high_confidence = all_faces_pts2(find(faces_normals_confidence(all_faces_pts2)==1));
            all_faces_pts3_high_confidence = all_faces_pts3(find(faces_normals_confidence(all_faces_pts3)==1));
            
            all_faces_high_confidence = [all_faces_pts1_high_confidence; all_faces_pts2_high_confidence; all_faces_pts3_high_confidence];
            
            if (size(all_faces_high_confidence,1) > 0)
                
                high_confidence_face_normal = faces_normals(all_faces_high_confidence(1),:);
                
                this_face_normal = faces_normals(i,:);
                
                this_angle = acosd(dot(this_face_normal, high_confidence_face_normal));
                
                
                if this_angle >= 130
                    faces_normals_confidence(i,1) = 1;
                    faces_normals(i,:) = -this_face_normal;
                elseif this_angle <=50
                    faces_normals_confidence(i,1) = 1;
                    faces_normals(i,:) = this_face_normal;
                else
                    disp(['Problem with ',num2str(i)])
                end
                
            end
        end
        
    end
    
end

vertices_normals = 0*vertices;

for i=1:1:size(vertices,1)
    %     disp(['Vertex Point :',num2str(i)])
    
    all_faces_pts = [find(faces(:,1)==i); find(faces(:,2)==i); find(faces(:,3)==i)];
    
    this_vertex_normal = mean(faces_normals(all_faces_pts,:));
    this_vertex_normal = this_vertex_normal/norm(this_vertex_normal);
    
    vertices_normals(i,:) = this_vertex_normal;
end

disp('Normal calculation Done!')

obj.BodyModel.shape.normals = vertices_normals;

% % test code for plotting normals
% 
% figure()
% hold on
% for i=1:1:length(ErosModel.BodyModel.shape.vertices)
%     plot3([ErosModel.BodyModel.shape.vertices(i,1) ErosModel.BodyModel.shape.vertices(i,1)+ErosModel.BodyModel.shape.normals(i,1)],...
%         [ErosModel.BodyModel.shape.vertices(i,2) ErosModel.BodyModel.shape.vertices(i,2)+ErosModel.BodyModel.shape.normals(i,2)],...
%         [ErosModel.BodyModel.shape.vertices(i,3) ErosModel.BodyModel.shape.vertices(i,3)+ErosModel.BodyModel.shape.normals(i,3)],...
%         '-k')
%     plot3(ErosModel.BodyModel.shape.vertices(i,1), ErosModel.BodyModel.shape.vertices(i,2), ErosModel.BodyModel.shape.vertices(i,3),'.r')
%     
% end
% hold off

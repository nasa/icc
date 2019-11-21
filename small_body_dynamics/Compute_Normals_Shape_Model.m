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

function obj = Compute_Normals_Shape_Model(obj)

disp('Start Normal calculation')

Vertices = obj.BodyModel.shape.vertices;
Faces = obj.BodyModel.shape.faces; 

Faces_Normals = 0*Faces;
Faces_Normals_Confidence = zeros(size(Faces,1),1);

iteration_number = 0;
disp(['Iteration :',num2str(iteration_number)])

for i=1:1:size(Faces,1)
    %     disp(['Iteration :',num2str(iteration_number),', Point :',num2str(i)])
    
    this_pts = Vertices(Faces(i,:),:);
    
    this_face_normal = cross(this_pts(2,:)-this_pts(1,:), this_pts(3,:)-this_pts(1,:));
    this_face_normal = this_face_normal/norm(this_face_normal);
    
    this_angle = acosd(dot(this_face_normal, this_pts(1,:)/norm(this_pts(1,:))));
    
    if this_angle >= 135
        Faces_Normals_Confidence(i,1) = 1;
        Faces_Normals(i,:) = -this_face_normal;
    elseif this_angle <=45
        Faces_Normals_Confidence(i,1) = 1;
        Faces_Normals(i,:) = this_face_normal;
    else
        Faces_Normals_Confidence(i,1) = 0.5;
        Faces_Normals(i,:) = this_face_normal;
    end
    
end

while sum(Faces_Normals_Confidence==0.5) > 0
    iteration_number = iteration_number + 1;
    disp(['Iteration :',num2str(iteration_number)])
    
    for i=1:1:size(Faces,1)
        
        if Faces_Normals_Confidence(i,1) == 0.5
            %             disp(['Iteration :',num2str(iteration_number),', Point :',num2str(i)])
            
            this_face_pts = Faces(i,:);
            
            all_faces_pts1 = [find(Faces(:,1)==this_face_pts(1)); find(Faces(:,2)==this_face_pts(1)); find(Faces(:,3)==this_face_pts(1))];
            all_faces_pts2 = [find(Faces(:,1)==this_face_pts(2)); find(Faces(:,2)==this_face_pts(2)); find(Faces(:,3)==this_face_pts(2))];
            all_faces_pts3 = [find(Faces(:,1)==this_face_pts(3)); find(Faces(:,2)==this_face_pts(3)); find(Faces(:,3)==this_face_pts(3))];
            
            all_faces_pts1_high_confidence = all_faces_pts1(find(Faces_Normals_Confidence(all_faces_pts1)==1));
            all_faces_pts2_high_confidence = all_faces_pts2(find(Faces_Normals_Confidence(all_faces_pts2)==1));
            all_faces_pts3_high_confidence = all_faces_pts3(find(Faces_Normals_Confidence(all_faces_pts3)==1));
            
            all_faces_high_confidence = [all_faces_pts1_high_confidence; all_faces_pts2_high_confidence; all_faces_pts3_high_confidence];
            
            if (size(all_faces_high_confidence,1) > 0)
                
                high_confidence_face_normal = Faces_Normals(all_faces_high_confidence(1),:);
                
                this_face_normal = Faces_Normals(i,:);
                
                this_angle = acosd(dot(this_face_normal, high_confidence_face_normal));
                
                
                if this_angle >= 135
                    Faces_Normals_Confidence(i,1) = 1;
                    Faces_Normals(i,:) = -this_face_normal;
                elseif this_angle <=45
                    Faces_Normals_Confidence(i,1) = 1;
                    Faces_Normals(i,:) = this_face_normal;
                else
                    disp(['Problem with ',num2str(i)])
                end
                
            end
        end
        
    end
    
end

Vertices_Normals = 0*Vertices;

for i=1:1:size(Vertices,1)
    %     disp(['Vertex Point :',num2str(i)])
    
    all_faces_pts = [find(Faces(:,1)==i); find(Faces(:,2)==i); find(Faces(:,3)==i)];
    
    this_vertex_normal = mean(Faces_Normals(all_faces_pts,:));
    this_vertex_normal = this_vertex_normal/norm(this_vertex_normal);
    
    Vertices_Normals(i,:) = this_vertex_normal;
end

disp('Normal calculation Done!')

obj.BodyModel.shape.normals = Vertices_Normals;

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

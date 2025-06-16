% function [ map ] = gen_square3d( square, map, draw )
%     if ~exist('draw', 'var')
%         draw = 0;
%     end
%     vert = [square(1,1) square(2,1) square(3,1); ...
%             square(1,2) square(2,1) square(3,1); ...
%             square(1,2) square(2,2) square  (3,1); ...
%             square(1,1) square(2,2) square(3,1); ...
%             square(1,1) square(2,1) square(3,2); ...
%             square(1,2) square(2,1) square(3,2); ...
%             square(1,2) square(2,2) square(3,2); ...
%             square(1,1) square(2,2) square(3,2)];
%     fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
% 
% 
%     if draw == 1
%         patch('Vertices',vert,'Faces',fac,...
%               'FaceVertexCData',hsv(6),'FaceColor','flat')
%         hold on
%     end
% 
%     for x = square(1,1):square(1,2)-1
%         for y = square(2,1):square(2,2)-1
%             for z = square(3,1):square(3,2)-1
%                 map(x,y,z) = 1;
%             end
%         end
%     end
% 
% 
% end
% 

function [ map ] = gen_square3d( limits, special_maze )
    % Generate a 3D map with walls and holes based on special_maze
    
    map = zeros(limits(1,2) - limits(1,1) + 1, ...
                limits(2,2) - limits(2,1) + 1, ...
                limits(3,2) - limits(3,1) + 1);
    
    % Create walls with holes based on special_maze pattern
    for x = 1:(limits(1,2) - limits(1,1) + 1)
        for y = 1:(limits(2,2) - limits(2,1) + 1)
            for z = 1:(limits(3,2) - limits(3,1) + 1)
                % Use special_maze to determine if this should be a wall (1) or hole (0)
                maze_x = mod(x-1, size(special_maze, 1)) + 1;
                maze_y = mod(y-1, size(special_maze, 2)) + 1;
                map(x, y, z) = special_maze(maze_x, maze_y);
            end
        end
    end
end
function [matches] = match(sift1, sift2)
%match - This function matches two buntches of SIFT features by seeking the nearest neighbor of each feature.
%Consulted material:
%[1] - http://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf
%in [1] it is recommended also to take into account the second nearest neighbour and ignore it if the distance is more than 0.8 between these two neighbours
% INPUT: 
% SIFT feature of the first image: sift1[128*n]
% SIFT feature of the second image: sift2[128*m]
% n may or may not equal to m
% OUTPUT:
% matches: M * 2 matrix, each row represents a match [index of p1, index of p2]


%
% Syntax: matches = match(sift1, sift2)
%

    num_1 = size(sift1,2);
    num_2 = size(sift2,2);
    nn_list = [];

    for i = 1:num_1
        nn_1 = -1;
        nn_2 = -1;
        min_1 = 999999;
        min_2 = 999999;
        a = sift1(:,i);
        for j = 1:num_2       
            b = sift2(:,j);
            dist = sqrt(sum((a-b).^2));

            if dist < min_1
                nn_1 = j;
                min_1 = dist;
            elseif dist < min_2
                nn_2 = j;
                min_2 = dist;   
            end
        end

        if min_1/min_2 <= 0.8
            nn_list = [nn_list;[i,nn_1]];
        end
    end

    matches = nn_list;

end
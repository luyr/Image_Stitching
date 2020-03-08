function H = RANSACFit(p1, p2, match, maxIter, seedSetSize, maxInlierError, goodFitThresh )
%RANSACFit Use RANSAC to find a robust affine transformation
% Input:
%   p1: N1 * 2 matrix, each row is a point
%   p2: N2 * 2 matrix, each row is a point
%   match: M * 2 matrix, each row represents a match [index of p1, index of p2]
%   maxIter: the number of iterations RANSAC will run
%   seedNum: The number of randomly-chosen seed points that we'll use to fit
%   our initial circle
%   maxInlierError: A match not in the seed set is considered an inlier if
%                   its error is less than maxInlierError. Error is
%                   measured as sum of Euclidean distance between transformed
%                   point1 and point2. You need to implement the
%                   ComputeCost function.
%
%   goodFitThresh: The threshold for deciding whether or not a model is
%                  good; for a model to be good, at least goodFitThresh
%                  non-seed points must be declared inliers.
%
% Output:
%   H: a robust estimation of affine transformation from p1 to p2
%
%
    N = size(match, 1);
    if N<3
        error('not enough matches to produce a transformation matrix')
    end
    if ~exist('maxIter', 'var'),
        maxIter = 600;
    end
    if ~exist('seedSetSize', 'var'),
        seedSetSize = ceil(0.2 * N);
        %seedSetSize = 100;
    end
    seedSetSize = max(seedSetSize,3);
    if ~exist('maxInlierError', 'var'),
        maxInlierError = 100;
    end
    if ~exist('goodFitThresh', 'var'),
        goodFitThresh = floor(0.7 * N);
        %goodFitThresh = 700;
    end
    H = eye(3);
% Here you implement basic RANSAC algorithm to compute the transformation H.
% You may need ConputError function bellow.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 YOUR CODE HERE                               %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    min_error = 999999;
    for i=1:maxIter
        
        inliers = randi(size(p1,1), 1, seedSetSize);
        %size(p1)
        H = ComputeAffineMatrix(p1(inliers,:), p2(inliers,:));
%         pt1 = p1;
%         pt1(inliers,:) = [];
%         pt2 = p2;
%         pt2(inliers,:) = [];
%         match(inlierrs,:) = [];
        errors = ComputeError(H, p1, p2, match);
        %errors
        
        % update inliers
        for i = 1:size(errors,1)
            if (errors(i,1) <= maxInlierError) && (size(find(inliers==i),2)==0)
                inliers = [inliers, i];
            end
        end
        
        % if goodfit, push into good_fit
        size(inliers);
        
        if size(inliers,2) >= goodFitThresh
            H = ComputeAffineMatrix(p1(inliers,:), p2(inliers,:));
            errors = ComputeError(H, p1, p2, match);
            if sum(errors) < min_error
                best_fit = H;
                min_error = sum(errors);
            end
        end
        
        
        
    end
    
    H = best_fit;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                             END OF YOUR CODE                                 %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if sum(sum((H - eye(3)).^2)) == 0
        disp('No RANSAC fit was found.')
    end
end

function dists = ComputeError(H, pt1, pt2, match)
% Compute the error using transformation matrix H to
% transform the point in pt1 to its matching point in pt2.
%
% Input:
%   H: 3 x 3 transformation matrix where H * [x; y; 1] transforms the point
%      (x, y) from the coordinate system of pt1 to the coordinate system of
%      pt2.
%   pt1: N1 x 2 matrix where each ROW is a data point [x_i, y_i]
%   pt2: N2 x 2 matrix where each ROW is a data point [x_i, y_i]
%   match: M x 2 matrix, each row represents a match [index of pt1, index of pt2]
%
% Output:
%    dists: An M x 1 vector where dists(i) is the error of fitting the i-th
%           match to the given transformation matrix.
%           Error is measured as the Euclidean distance between (transformed pt1)
%           and pt2 in homogeneous coordinates.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                YOUR CODE HERE.                               %
%           Convert the points to a usable format, perform the                 %
%           transformation on pt1 points, and find their distance to their     %
%           MATCHING pt2 points.                                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % hint: If you have an array of indices, MATLAB can directly use it to
    % index into another array. For example, pt1(match(:, 1),:) returns a
    % matrix whose first row is pt1(match(1,1),:), second row is
    % pt1(match(2,1),:), etc. (You may use 'for' loops if this is too
    % confusing, but understanding it will make your code simple and fast.)
    
    
    dists = [];

    pt1 = [pt1, ones(size(pt1,1),1)]';
    pt2 = [pt2, ones(size(pt2,1),1)]';
    size(pt1);
    pt1_T = H*pt1;
    for i = 1:size(pt1_T,2)
        %pt1(:,i)
        dists = [dists;sqrt(sum((pt1_T(:,i)-pt2(:,i)).^2))];
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 END YOUR CODE                                %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if size(dists,1) ~= size(match,1) || size(dists,2) ~= 1
        error('wrong format');
    end
end

function [D1, D2] = part(D, splitSize)
    idx = randperm(size(D, 1));
    D1 = D(idx(1:splitSize), :);
    D2 = D(idx(splitSize+1:end), :);
end

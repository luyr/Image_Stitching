clear all;
% Set up VLfeat Toolbox here.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 YOUR CODE HERE                               %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


run('./vlfeat-0.9.20/toolbox/vl_setup')
% Read in the images that you want to stitch together.
% Better to transform RGB images into gray scale images. vl_sift requires 'single' type.
disp('reading img');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 YOUR CODE HERE                               %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

image1 = imread('demo2_1.jpg');
pic_1 = imread('demo2_1.jpg');
pic_1_shape = size(pic_1);
rows = pic_1_shape(1);
cols = pic_1_shape(2);
scale_factor = 500/cols;
image1 = imresize(image1, scale_factor, 'bilinear');
pic_1 = imresize(pic_1, scale_factor, 'bilinear');
pic_1 = single(rgb2gray(pic_1));

image2 = imread('demo2_2.jpg');
pic_2 = imread('demo2_2.jpg');
pic_2_shape = size(pic_2);
rows = pic_2_shape(1);
cols = pic_2_shape(2);
scale_factor = 500/cols;
image2 = imresize(image2, scale_factor, 'bilinear');
pic_2 = imresize(pic_2, scale_factor, 'bilinear');
pic_2 = single(rgb2gray(pic_2));


% imshow(pic_1)


% Obtaininig SIFT Correspondences by VLfeat tool box.
% Btw, what's the meaning of the outputs?
% keep track of feature points coordinates.
disp('Obtaininig SIFT Correspondences');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 YOUR CODE HERE                               %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[f1,d1] = vl_sift(pic_1);
[f2,d2] = vl_sift(pic_2);
%size(f1)

% Here you should matching the SIFT feature between adjacent images by L2 distance. 
% Please fill the function match.m
disp('matching keypoints');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 YOUR CODE HERE                               %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

matches = match(d1, d2);
% size(matches1)
% [matches,scores] = vl_ubcmatch(d1,d2);
% matches = matches';
% size(matches)
%plotMatches(image1,image2,matches)

% Estimating Homography using RANSAC
% Please fill the function RANSACFit.m
disp('RANSACing');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                              %
%                                 YOUR CODE HERE                               %
%                                                                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%(
shape = size(matches);
num_matches = shape(1);
matches_1 = matches(:,1);
matches_2 = matches(:,2);
p1 = [];
p2 = [];
match = [];
for i = 1: num_matches
    p1 = [p1; f1(1:2, matches_1(i))'];
    p2 = [p2; f2(1:2, matches_2(i))'];
    match = [match; [i,i]];
end

H = RANSACFit(p1,p2,match);


%% Creating the panorama
% use cell() to store correspondent images and transformations
disp('Generating panorama pictures...');
IMAGES = {image1,image2};
TRANS = {H};
MultipleStitch(IMAGES, TRANS, 'demo2.jpg' );
%PairStitch(image1, image2, H, 'demo2.jpg');

% In plotMatches.m you can visualize the matching results after you feed proper data stream. Feel free to create your own visualization.

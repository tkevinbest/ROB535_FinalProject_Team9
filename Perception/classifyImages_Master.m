clear
close all

% Load the images
imageFolder_Training = uigetdir('Locate the training image folder on your computer:');
trainLabels = readtable((fullfile(imageFolder_Training, 'labels.csv')));
trainingImages = imageDatastore(imageFolder_Training,'IncludeSubfolders',true); 
trainingImages.Labels = trainLabels.label;


%% Examples to make sure things worked
% Make a subplot of the first 3 types of images
showImages = true; 
if showImages
    unknown = find(trainingImages.Labels == 0, 1); 
    cars = find(trainingImages.Labels == 1, 1); 
    motorcycles = find(trainingImages.Labels == 2, 1); 
    
    subplot(1,3,1)
    imshow(readimage(trainingImages, unknown))
    subplot(1,3,2)
    imshow(readimage(trainingImages, cars))
    subplot(1,3,3)
    imshow(readimage(trainingImages, motorcycles))
end


%% Preprocess data - balance if desired


%% Load pretrained network
featureNetwork = resnet50(); 


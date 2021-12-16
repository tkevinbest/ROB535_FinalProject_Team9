function writeCentroidOutputFile(fileNames, centroids)

% Remove the beginning string so we're just left with the file name
fileNamesShort = extractAfter(fileNames, strcat('test', filesep));
% Kaggle wants the other slash so we'll give it to it. Also remove the word
% image
fileNamesShort = strrep(strrep(fileNamesShort, '\','/'),'_image.jpg','/{}');
fileNamesX = strrep(fileNamesShort, '{}', 'x');
fileNamesY = strrep(fileNamesShort, '{}', 'y');
fileNamesZ = strrep(fileNamesShort, '{}', 'z');

fileNamesOutput = [fileNamesX, fileNamesY, fileNamesZ]';
fileNamesOutput = fileNamesOutput(:);
centroidsOutput = centroids(:);

outTable = table(['guid/image/axis';fileNamesOutput], ['value';string(centroidsOutput)]);
writetable(outTable, 'testCentroids.csv', 'WriteVariableNames',0);
end
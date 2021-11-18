function writeOutputFile(fileNames, labels)

% Remove the beginning string so we're just left with the file name
fileNamesShort = extractAfter(fileNames,'test\');
% Kaggle wants the other slash so we'll give it to it. Also remove the word
% image
fileNamesShort = strrep(strrep(fileNamesShort, '\','/'),'_image.jpg','');

outTable = table(['guid/image';fileNamesShort], ['label';string(labels)]);
writetable(outTable, 'testLabels.csv', 'WriteVariableNames',0);
end
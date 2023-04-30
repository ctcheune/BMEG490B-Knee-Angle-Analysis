%File: importMyRIO.m
%Author: Liam Foulger
%Date Created: 2022-05-01
%Last Updated: 2022-09-02
%
%rawData = importMyRIO(selpath)
%
%Function to get the data from a trial folder from the myRIO device, and
%collate all the minute folders into a single matrix that is correctly
%temporally ordered
%Input:
%-path of trial data. This will be a folder with each minute subfolder
%within
%Output:
%-rawData: collated raw data, where rows are time points and columns are
%different data measures from myRIO
%
%Adapted from code by Anthony Chen

function [rawData, trialName, calibrationData] = importMyRIO(namePairArguments)
    arguments
        namePairArguments.Path string = ""
        namePairArguments.Calibration string = ""

    end
    if strcmp(namePairArguments.Path,"")
        selpath = uigetdir('','Get Trial Data'); %get folder with trial data
    else
        selpath = namePairArguments.Path;
    end

    D = char(selpath);
    S = dir(fullfile(D,'*'));
    N = setdiff({S.name},{'.','..'}) ;
    %resort N by minute #, not alphabetical

    N = {N{contains(N,'Minute')}};

    for ii = 1:numel(N)
        idx = sscanf(N{ii},'Minute_%d');
        N_sorted{idx} = N{ii};

    end

    %remove empty cells
    N_sorted = N_sorted(~cellfun('isempty',N_sorted));

    rawData = [];
    for ii = 1:numel(N)
        T = fullfile(D,N_sorted{ii});

        minuteData{ii} = readmatrix(T);
        rawData = [rawData; minuteData{ii}];
    end
    
    calibFind = strfind(selpath,"\");
    trialName = selpath( (calibFind(end)+1):end );
    
    if strcmp(namePairArguments.Calibration,"")
        calibrationData = [];
    else
        if namePairArguments.Calibration == "Get"
            [infiles, inpath] = uigetfile({'*calibration*','*.txt*'},'Get Calibration',selpath(1:(calibFind(end)-1))); %get .mat file with trial data
            file_path = fullfile(inpath, infiles);
        else
            file_path = namePairArguments.Calibration;
        end
        calibrationData = readmatrix(file_path);
    end
end
clc
clear
close all;
imgL =imread('data\scene1.row3.col3.ppm');% imread('data\test.jpg');%imread('data\scene1.row3.col3.ppm');
imgL = rgb2gray(imgL);

imgR =imread('data\scene1.row3.col4.ppm');% imread('data\test.jpg');%imread('data\scene1.row3.col3.ppm');
imgR = rgb2gray(imgR);



s = 1;  %步长
% 计算x,y方向梯度
[pxL,pyL] = gradient_0(imgL,s);
[pxR,pyR] = gradient_0(imgR,s);
%Magnitude_xy = sqrt(pxL.^2+py.^2);
MagThresh = 20;
SimThresh = 0.7*MagThresh;
xDiffThresh = 40;
tic;
%% 提取特征点
fea_xL = [];
fea_yL = [];
for i=1:size(pxL,1)
    for j=1:size(pxL,2)
        %if (Magnitude_xy(i,j)> MagThresh  ...
           %    && abs(px(i,j)- py(i,j))<SimThresh )
        if (abs(pxL(i,j)) > xDiffThresh)
            fea_xL = [fea_xL,j];
            fea_yL = [fea_yL,i];
        end
    end
end
imshow(imgL);
hold on;
plot(fea_xL,fea_yL,'r.');

fea_xR = [];
fea_yR = [];
for i=1:size(pxR,1)
    for j=1:size(pxR,2)
        %if (Magnitude_xy(i,j)> MagThresh  ...
           %    && abs(px(i,j)- py(i,j))<SimThresh )
        if (abs(pxR(i,j)) > xDiffThresh)
            fea_xR = [fea_xR,j];
            fea_yR = [fea_yR,i];
        end
    end
end
figure;
imshow(imgR);
hold on;
plot(fea_xR,fea_yR,'r.');


%% matching
%邻接表建立
imgH = size(imgL,1);
imgW = size(imgL,2);

fea_list_L = cell(length(fea_xL), 1);
for i=1:length(fea_xL)
    x = fea_xL(i);
    y = fea_yL(i);
    fea_list_L{y} = [fea_list_L{y},x];
end

fea_list_R = cell(length(fea_xR), 1);
for i=1:length(fea_xR)
    x = fea_xR(i);
    y = fea_yR(i);
    fea_list_R{y} = [fea_list_R{y},x];
end



%交叉匹配 未加peak ratio
winSize = 5; % odd 
winArea = winSize*winSize;
halfWinSize = floor(winSize / 2 );
pair = [];
for y = 1:size(fea_list_L,1)
    
     if (y<=halfWinSize || y>imgH - halfWinSize || x<=halfWinSize || x >imgW - halfWinSize ) 
          continue;
     end
     if (isempty(fea_list_L{y}) || isempty(fea_list_R{y}) )
        continue;
     end
     
   dist = [];%注意清空
   for i = 1:length(fea_list_L{y}) 
       xL = fea_list_L{y}(i);
       for j = 1:length(fea_list_R{y})
           xR= fea_list_R{y}(j);
          
           SAD = double( imgL(y-halfWinSize:y+halfWinSize,xL-halfWinSize:xL+halfWinSize) ) - ...
                    double( imgR(y-halfWinSize:y+halfWinSize,xR-halfWinSize:xR+halfWinSize) );
                
           SAD = sum(sum( abs(SAD) ));
            
           dist(i,j) = SAD/winArea;
           
          %{ 
           SAD = 0;
           for win_i = 1:winSize
               for win_j = 1:winSize
                   x_r = xR + win_j - int(winSize)*0.5;
                   y = y  + win_i - int(WinSize)*0.5;
                   x_l = xL + win_j - int(winSize)*0.5;
                   
                   SAD = SAD + imgL
                     dist(i,j) = 
               end
           end
           %}
       end
   end
    % 排序，并且选出交叉验证
        [L_R,I_lr] = sort(dist,2);
        [R_L,I_rl] = sort(dist,1);
        for i = 1:length(fea_list_L{y})
            if ( I_rl(1,I_lr(i,1)) == i)  %%%% && L_R(i,1)<6
               pair = [pair; fea_list_L{y}(i),y, fea_list_R{y}(I_lr(i,1)), y];
            end
        end
        
end
 toc;
twoImg = [imgL,imgR];
figure;
imshow(twoImg);
hold on;
for i=1:10:size(pair,1)
        plot([pair(i,1),pair(i,3)+imgW], [pair(i,2),pair(i,4)],'.');
      plot([pair(i,1),pair(i,3)+imgW], [pair(i,2),pair(i,4)],'-');
end
    


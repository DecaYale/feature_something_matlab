
function [px_,py_] = gradient_45(img,s)
%45度梯度，s是步长
img =  double(img);
rows= size(img,1);
cols = size(img,2);
px_ = zeros(rows,cols);
py_ = zeros(rows,cols);
sqrt2 = sqrt(2);

for i=1:rows
    for j=1:cols
        if (i<=s||j<=s||i>=rows-s+1||j>=cols-s+1)
           continue;
        end
        px_(i,j) = ( img(i+s,j+s) - img(i-s,j-s) )/(2*s*sqrt2);
        py_(i,j) = ( img(i+s,j-s) - img(i-s,j+s) )/(2*s*sqrt2);
    end
end

end
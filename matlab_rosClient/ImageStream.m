function ImageStream( A,Params,handle )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
Img = mex_mmap('readimage',A,'camera1');
Img2 = reshape([Img(Params.indexR);Img(Params.indexG);Img(Params.indexB)],Params.width,Params.height,3);
if nargin == 3
    set(handle,'CData',Img2);%Set the Image as CData
    drawnow;
else
    imshow(Img2);
end

end


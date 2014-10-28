%Image Streaming:
Params.width = 320;
Params.height = 240;
Params.indexR = (1:3:(Params.width*Params.height*3));
Params.indexG = Params.indexR + 1;
Params.indexB = Params.indexG + 1;
A = mex_mmap('new');
mex_mmap('loadcamera',A,'camera1');
%%
t = timer('TimerFcn',{@(~,~,x,y)ImageStream(x,y),A,Params},'Period',0.05,'ExecutionMode','fixedRate');
start(t);
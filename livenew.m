vid=videoinput('winvideo',1,'MJPG_1280x720');
 
set(vid,'ReturnedColorSpace','rgb');       
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
triggerconfig(vid,'manual');
%Capture one frame per trigger
 
set(vid,'FramesPerTrigger',1 );
set(vid,'TriggerRepeat', Inf);
start(vid); %start video
aa=1;
%Infinite while loop
Out=[];
while(1)
% preview(vid)
trigger(vid);
%Get Image
im=getdata(vid,1);
imshow(im);
hold on
if aa == 5
red=im(:,:,1);
Green=im(:,:,2);
Blue=im(:,:,3);
Out(:,:,1)=red;
Out(:,:,2)=Green;
Out(:,:,3)=Blue;
Out=uint8(Out);
 
end
 
if aa > 5
red=im(:,:,1);
Green=im(:,:,2);
Blue=im(:,:,3);
red1=Out(:,:,1);
Green1=Out(:,:,2);
Blue1=Out(:,:,3);
z1 = imabsdiff(red,red1);     %get absolute diffrence between both images
z2 = imabsdiff(Green,Green1); %get absolute diffrence between both images
z3 = imabsdiff(Blue,Blue1);   %get absolute diffrence between both images
zz1= sum(z1,1);               %calculate SAD
zz2= sum(z2,1);               %calculate SAD
zz3= sum(z3,1);               %calculate SAD
zzz1= sum(zz1)/ 307200;
zzz2= sum(zz2)/ 307200;
zzz3= sum(zz3)/ 307200;
Final=(zzz1+zzz2+zzz3)/3
disp(Final);
   
end
aa=aa+1;
 
disp(aa);
if aa == 100
   break
end
end
stop(vid),delete(vid),clear vid;
 
%- See more at: https://www.pantechsolutions.net/matlab-code-for-real-time-motion-detection#sthash.Frt1AcN1.dpuf
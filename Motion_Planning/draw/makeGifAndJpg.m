function makeGifAndJpg(fig_num)
    
    saveas(figure(4), 'postion','jpg');
    pause(0.1);
    saveas(figure(5), 'velocity','jpg');
    pause(0.1);
    filename = 'testAnimated6.gif';
    
    h = figure(fig_num);
    for i = 1:90
        view(4*i+15, 15);
        drawnow 

        % Capture the plot as an image 
        frame = getframe(h); 
        im = frame2im(frame); 
        [imind,cm] = rgb2ind(im,256); 

        % Write to the GIF File 
        if i == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
        else 
          imwrite(imind,cm,filename,'gif','WriteMode','append'); 
        end 
    end

end


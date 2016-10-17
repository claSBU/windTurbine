clc
close all
clear all

%% Generate Path %%
tic;
x = 0:.1:10;
y = 0:.1:10;

x_agent = [1];
y_agent = [1];

Act = zeros(length(x),length(y));
V = zeros(length(x),length(y));

[xx,yy] = meshgrid(x,y);

figure;
plot(xx,yy,'k.')
hold on;
x_goal = 9;
y_goal = 9;


x_start = 0;
y_start = 0;

[i_x,i_y]= xy_to_indices(x_goal,y_goal);
ix_goal = i_x;
iy_goal = i_y;

i_do_nth = [i_x,i_y];

Act(i_x,i_y) = 0;
plot(x_goal,y_goal,'ro')

obs_locs = [;
    2 2  4 8;
    4 0.25  8 1;
    6 1  8 6;
    6 7 9.5 8;
    2 7 5 9;
    7 8 8 9.5;
    4 2 5.5 4
    ];

for i = 1:size(obs_locs,1)
    patch( [obs_locs(i,1) obs_locs(i,1)  obs_locs(i,3) obs_locs(i,3)  ], ...
        [obs_locs(i,2) obs_locs(i,4)  obs_locs(i,4) obs_locs(i,2)  ],'g' );
    
    [ix_obs_st,iy_obs_st]= xy_to_indices( obs_locs(i,1), obs_locs(i,2));
    [ix_obs_en,iy_obs_en]= xy_to_indices( obs_locs(i,3), obs_locs(i,4));
    
    Act(ix_obs_st:ix_obs_en,iy_obs_st:iy_obs_en) =  1000;
end


close all
figure;
V = 2000*ones(size(Act));
gam = .9;
filename = ['Value_growth' num2str(i) '.gif'];

changed = 1;
i_V = 1;
while changed == 1
    changed = 0;
    V_old = V;
    for i_x = 1:length(x)
        for i_y = 1:length(y)
            
            if (i_x == ix_goal) &&(i_y == iy_goal)
                if V(i_x,i_y) > 0
                    changed = 1;
                    V(i_x,i_y) = 0;
                end
            end
            
            if Act(i_x,i_y) ~= 1000;
                iv_x = [1 -1 0 0 1 -1 1  -1];
                iv_y = [0 0 -1 1  1 -1 -1 1];
                V_new = [];
                for i_v = 1:8,
                    val = check_ind(i_x+iv_x(i_v),i_y+iv_y(i_v));
                    if val == 1
                        V_new  = V(i_x+iv_x(i_v),i_y+iv_y(i_v)) + 10*sqrt(iv_x(i_v)^2+iv_y(i_v)^2);
                        
                        if V_new< V(i_x,i_y)
                            V(i_x,i_y) = V_new;
                            changed = 1;
                            
                        end
                    end
                end
                
                
            else
                V(i_x,i_y) = 2000;
            end
            
            
            
            
            
        end
    end
    
    
    surf(yy,xx,V_old);xlabel('X');ylabel('Y');zlabel('Value')
    title(['Value after ' num2str(i_V) ' iterations.'])
    axis([0 10 0 10 -200 2100])
    view(i_V*3,30);
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i_V == 1;
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.1);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.1);
    end
    i_V = i_V+1;
end


for i = i_V:i_V+60
    surf(yy,xx,V);xlabel('X');ylabel('Y');zlabel('Value')
    title(['Value after ' num2str(i) ' iterations (Converged).'])
    axis([0 10 0 10 -200 2100])
    view(i*3,30);
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.1);
    
end



filename = ['Obs_Avoidance' num2str(i) '.gif'];
close all

figure;
plot(xx,yy,'k.')
hold on;
for i = 1:size(obs_locs,1)
    patch( [obs_locs(i,1) obs_locs(i,1)  obs_locs(i,3) obs_locs(i,3)  ], ...
        [obs_locs(i,2) obs_locs(i,4)  obs_locs(i,4) obs_locs(i,2)  ],'g' );
    
end

Va = [];
hold on;
i_move = 1;
m = 0;
path=[];

for i = 1:length(x_agent)
    [i_x,i_y]= xy_to_indices(x_agent(i),y_agent(i));
    stop_mov = 0;
    while stop_mov == 0
        m = m+1;
        iv_x = [1 -1 0 0 1 -1 1  -1];
                iv_y = [0 0 -1 1  1 -1 -1 1];
        for i_v = 1:8,
            Va(i_v) = V(i_x+iv_x(i_v),i_y+iv_y(i_v)) + 10*sqrt(iv_x(i_v)^2+iv_y(i_v)^2);
        end
        
        [V_min , i_vmin]= min(Va);
        x_agent(i) = x( i_x+iv_x(i_vmin));
        y_agent(i) = y( i_y+iv_y(i_vmin));
        path(m,:) = [x_agent y_agent];
        plot(x_agent(i),y_agent(i),'bx')
        plot(x_agent(i),y_agent(i),'b*')
        
        if (i_x==ix_goal)&&(i_y==iy_goal)
            stop_mov = 1;
        end
        
        frame = getframe(1);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i_move == 1;
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',.1);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',.1);
        end
        i_move = i_move+1;
        
        [i_x,i_y]= xy_to_indices(x_agent(i),y_agent(i));
       
        pause(0.01);
    end
end
toc;

%% Smooth Path %%
tic;
path
newPath = zeros(length(path),2);
newPath(1,:) = path(1,:);
newPath(end,:)=path(end,:);

for i = 2:length(path)-1
    newPath(i,:) = newPath(i,:)+0.5*(path(i,:)-newPath(i,:))+0.1*(newPath(i-1,:)-2*newPath(i,:)+newPath(i+1,:));
end

toc;
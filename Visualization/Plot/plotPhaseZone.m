% plot phase zone
prev_t = 0;
for i = 1:n
    if i == n
        if DS(n) == 2 || DS(n) == 5
            hold on 
            fill([prev_t prev_t T(i) T(i)],[min_height,max_height,max_height,min_height], [0.9 0.9 0.9], 'EdgeColor',[0.9 0.9 0.9]);
            hold off
        elseif DS(n) == 3 || DS(n) == 6
            hold on 
            fill([prev_t prev_t T(i) T(i)],[min_height,max_height,max_height,min_height], [0.8 0.8 0.9], 'EdgeColor',[0.8 0.8 0.9]);
            hold off
        end
    else
        if DS(i)~=DS(i+1)
            if DS(i) == 1 || DS(i) == 4  
                prev_t = T(i+1);
            elseif DS(i) == 2 || DS(i) == 5 
                hold on 
                fill([prev_t prev_t T(i+1) T(i+1)],[min_height,max_height,max_height,min_height], [0.9 0.9 0.9], 'EdgeColor',[0.9 0.9 0.9]);
                hold off
                prev_t = T(i+1);
            elseif DS(i) == 3 || DS(i) == 6
                hold on 
                fill([prev_t prev_t T(i+1) T(i+1)],[min_height,max_height,max_height,min_height], [0.8 0.8 0.9], 'EdgeColor',[0.8 0.8 0.9]);
                hold off
                prev_t = T(i+1);
            end
        end
    end
end
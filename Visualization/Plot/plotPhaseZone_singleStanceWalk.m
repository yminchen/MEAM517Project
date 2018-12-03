% plot phase zone

n = numel(T);

prev_t = 0;
i_start = 1;
for i = 1:n
    if i == n
        if DS(n) == 2 || DS(n) == 3
            hold on 
            fill([T(i_start) T(i_start) T(i) T(i)],[min_height,max_height,max_height,min_height], [0.9 0.9 0.9], 'EdgeColor',[0.9 0.9 0.9]);
            hold off
        elseif DS(n) == 5 || DS(n) == 6
            hold on 
%             fill([T(i_start) T(i_start) T(i) T(i)],[min_height,max_height,max_height,min_height], [0.8 0.8 0.9], 'EdgeColor',[0.8 0.8 0.9]);
            hold off
        else
            disp('Careful. Phase zone plot is not correct. The phase contains flight or double stance');
        end
    else
        if DS(i)~=DS(i+1)
            if DS(i) == 2 || DS(i) == 3 
                hold on 
                fill([T(i_start) T(i_start) T(i+1) T(i+1)],[min_height,max_height,max_height,min_height], [0.9 0.9 0.9], 'EdgeColor',[0.9 0.9 0.9]);
                hold off
                i_start = i+1;
            elseif DS(i) == 5 || DS(i) == 6
                hold on 
%                 fill([T(i_start) T(i_start) T(i+1) T(i+1)],[min_height,max_height,max_height,min_height], [0.8 0.8 0.9], 'EdgeColor',[0.8 0.8 0.9]);
                hold off
                i_start = i+1;
            else 
                disp('Careful. Phase zone plot is not correct. The phase contains flight or double stance');
            end
        end
    end
end
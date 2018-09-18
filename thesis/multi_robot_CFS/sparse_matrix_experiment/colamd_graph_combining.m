function [] = colamd_graph_combining(episodes_per_problem, matrix_id, plot_fancy_results)


sizes = zeros(1, length(matrix_id) * episodes_per_problem);
dist = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_raw = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_colamd = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_myorder = zeros(1, length(matrix_id) * episodes_per_problem);

time_raw = zeros(1, length(matrix_id) * episodes_per_problem);
time_colamd = zeros(1, length(matrix_id) * episodes_per_problem);
time_myorder = zeros(1, length(matrix_id) * episodes_per_problem);

if nargin == 2
    plot_fancy_results = false;
end

if plot_fancy_results
    assert(episodes_per_problem == 1);
    figure;
end


for sz = 1:length(matrix_id)
    problem = UFget(matrix_id(sz));

    input_matrix = problem.A;
    if size(input_matrix, 1) < 2 * size(input_matrix, 2)
        input_matrix = input_matrix(:, 1:round(size(input_matrix, 2)/2.5)); % 2.5 is some number greater than 2.
    end
    input_matrix_size = size(input_matrix,2);
    
    input_matrix_A = input_matrix(1:floor(size(input_matrix,1)/2), :);
    input_matrix_B = input_matrix(ceil(size(input_matrix,1)/2):size(input_matrix,1), :);
    input_matrix_size_A = size(input_matrix_A,2); % TODO: Delete these vars
    input_matrix_size_B = size(input_matrix_B,2);
    
    assert(input_matrix_size == input_matrix_size_A);
    assert(input_matrix_size == input_matrix_size_B); 
    
%     input_matrix_order = colamd(input_matrix);
%     R = qr(input_matrix);

    input_matrix_order_A = colamd(input_matrix_A);
    input_matrix_order_B = colamd(input_matrix_B);
    R_A = qr(input_matrix_A);
    R_B = qr(input_matrix_B);

    i = 1;
    while i <= episodes_per_problem
        sample_init_size_A = randi([round(input_matrix_size/20) round(input_matrix_size/10)], 1);
        sample_init_size_B = randi([round(input_matrix_size/20) round(input_matrix_size/10)], 1);
        random_pick_vars_A = randsample(input_matrix_size, sample_init_size_A)';
        random_pick_vars_B = randsample(input_matrix_size, sample_init_size_B)';
        
        total_affected_variables_A = random_pick_vars_A;
        total_affected_variables_B = random_pick_vars_B;

        k = 1;
        while k <= length(total_affected_variables_A)
            row_considered_binary = zeros(1, input_matrix_size);
            row_considered = R_A(total_affected_variables_A(k),:);
            row_considered_binary(row_considered ~= 0) = 1;
            assert(~any(total_affected_variables_A > input_matrix_size));
            for l = total_affected_variables_A(k)+1:input_matrix_size
                if row_considered_binary(l) > 0 && ~any(total_affected_variables_A == l)
                    total_affected_variables_A = [total_affected_variables_A l];
                end
            end
            k = k + 1;
        end
        
        k = 1;
        while k <= length(total_affected_variables_B)
            row_considered_binary = zeros(1, input_matrix_size);
            row_considered = R_B(total_affected_variables_B(k),:);
            row_considered_binary(row_considered ~= 0) = 1;
            assert(~any(total_affected_variables_B > input_matrix_size));
            for l = total_affected_variables_B(k)+1:input_matrix_size
                if row_considered_binary(l) > 0 && ~any(total_affected_variables_B == l)
                    total_affected_variables_B = [total_affected_variables_B l];
                end
            end
            k = k + 1;
        end 
        
        if isempty(intersect(total_affected_variables_A, total_affected_variables_B))
            continue;
        elseif isequal(total_affected_variables_A, total_affected_variables_B)
            continue;
        end
        
        sample_final_size = length(union(total_affected_variables_A, total_affected_variables_B));
%         sample_matrix_col_indices = randsample(input_matrix_size, sample_matrix_size);
%         sample_matrix_col_indices = total_affected_variables_A;
        pure_col_indices_A = setdiff(total_affected_variables_A, intersect(total_affected_variables_A, total_affected_variables_B));
        pure_col_indices_B = setdiff(total_affected_variables_B, intersect(total_affected_variables_A, total_affected_variables_B));
        common_col_indices = intersect(total_affected_variables_A, total_affected_variables_B);
        sample_matrix = blkdiag(input_matrix_A(:,pure_col_indices_A), input_matrix_B(:,pure_col_indices_B));
        for j = intersect(total_affected_variables_A, total_affected_variables_B)
            sample_matrix = [sample_matrix [input_matrix_A(:,j); input_matrix_B(:,j)]];
        end
        
%         sample_matrix = input_matrix(:, sample_matrix_col_indices);
        sample_matrix_colamd_order = colamd(sample_matrix);
        
        myorder_A = relative_ordering(input_matrix_order_A, pure_col_indices_A);
        myorder_B = relative_ordering(input_matrix_order_B, pure_col_indices_B);
        myorder_B = myorder_B + length(myorder_A);
        myorder_common = relative_ordering(input_matrix_order_A, common_col_indices);
        myorder_common = myorder_common + length(myorder_B) + length(myorder_A);
        myorder = [myorder_A myorder_B myorder_common];


        nnz_raw(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix(:, fliplr(sample_matrix_colamd_order))))/numel(sample_matrix);
        nnz_colamd(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix(:, sample_matrix_colamd_order)))/numel(sample_matrix);
        nnz_myorder(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix(:, myorder)))/numel(sample_matrix);

%         sizes(((sz-1) * episodes_per_problem) + i) = sample_final_size;
%         dist(((sz-1) * episodes_per_problem) + i) = pdist2(sample_matrix_colamd_order, sample_matrix_myorder, 'hamming');

        if episodes_per_problem == 1
            subplot(length(matrix_id), 4, (sz-1)*length(matrix_id) + 1);
            img = imread(strcat(num2str(matrix_id(sz)), '.png'));
            imagesc(img);
            axis off
            if sz == 1
                title({'Real World', 'Dataset'});
            end

            subplot(length(matrix_id), 4, (sz-1)*length(matrix_id) + 2);
            tic;
            [~, r_colamd] = qr(sample_matrix(:, sample_matrix_colamd_order));
            time_colamd(((sz-1) * episodes_per_problem) + i) = toc;
            spy(r_colamd(1:size(r_colamd,2),:));
            if sz == 1
                title('COLAMD');
            end

            subplot(length(matrix_id), 4, (sz-1)*length(matrix_id) + 3);
            tic;
            [~, r_myorder] = qr(sample_matrix(:, myorder));
            time_myorder(((sz-1) * episodes_per_problem) + i) = toc;
            spy(r_myorder(1:size(r_myorder,2),:));
            if sz == 1
                title({'Fusion', 'Ordering'});
            end        

            subplot(length(matrix_id), 4, (sz-1)*length(matrix_id) + 4);
            tic;
            [~, r_raw] = qr(sample_matrix(:, fliplr(sample_matrix_colamd_order)));
            time_raw(((sz-1) * episodes_per_problem) + i) = toc;
            spy(r_raw(1:size(r_raw,2),:));
            if sz == 1
                title({'w/o Ordering', '(as stored', 'in Memory)'});
            end 
        end
        
        tic;
        [~, r_colamd] = qr(sample_matrix(:, sample_matrix_colamd_order));
        time_colamd(((sz-1) * episodes_per_problem) + i) = toc;

        tic;
        [~, r_myorder] = qr(sample_matrix(:, myorder));
        time_myorder(((sz-1) * episodes_per_problem) + i) = toc;
            
        tic;
        [~, r_raw] = qr(sample_matrix(:, fliplr(sample_matrix_colamd_order)));
        time_raw(((sz-1) * episodes_per_problem) + i) = toc;
                
        i = i + 1;
    end
end

[~, sorted_index] = sort(time_raw);
figure;
hold on;
plot(time_raw(sorted_index), 'b.', 'MarkerSize', 10);
plot(time_colamd(sorted_index), 'r.', 'MarkerSize', 10);
plot(time_myorder(sorted_index), 'g.', 'MarkerSize', 10);

[~, sorted_index] = sort(nnz_raw);
figure;
hold on;
plot(nnz_raw(sorted_index), 'bd', 'MarkerSize', 10);
plot(nnz_colamd(sorted_index), 'rd', 'MarkerSize', 10);
plot(nnz_myorder(sorted_index), 'gd', 'MarkerSize', 10);

figure;
hold on;
stem(nnz_raw, 'filled', 'Marker', 'd', 'LineWidth', 0.75);
stem(nnz_colamd, 'filled', 'Marker', 'd', 'LineWidth', 0.75);
stem(nnz_myorder, 'filled', 'Marker', 'd', 'LineWidth', 0.75);

hold off;

end

function myorder = relative_ordering(colamd_order, query_indices)
        query_size = length(query_indices);
        pos_in_input_matrix = zeros(1, query_size);
        for j = 1:query_size
    %         pos_in_input_matrix(j) = input_matrix_order(sample_matrix_col_indices(j));
%     Logs the index in which the value query_indices(j) is present in
%     colamd_order array in pos_in_input_matrix(j)
            pos_in_input_matrix(j) = find(colamd_order == query_indices(j));
        end

%         [~, mlab_order] = sort(pos_in_input_matrix);
        myorder = zeros(1, query_size);
%         sample_matrix_myordera = zeros(1, sample_matrix_size);

        pos_in_input_matrix_copy = pos_in_input_matrix;

        for j = 1:query_size
            [~,ind] = min(pos_in_input_matrix);
            pos_in_input_matrix(ind) = inf;
            myorder(j) = ind;
    %         sample_matrix_myordera(j) = find(sample_matrix_col_indices(ind) == sample_matrix_col_indices);
        end
        
%         figure;
%         hold on;
%         plot(pos_in_input_matrix_copy, 'b');
%         plot(myorder, 'r');
%         clf;

    %     assert(isequal(sample_matrix_myordera, sample_matrix_myorder));

end



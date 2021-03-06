function [] = colamd_chop_verification(episodes_per_problem, matrix_id)


sizes = zeros(1, length(matrix_id) * episodes_per_problem);
dist = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_raw = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_colamd = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_myorder = zeros(1, length(matrix_id) * episodes_per_problem);

for sz = 1:length(matrix_id)
    problem = UFget(matrix_id(sz));

    input_matrix = problem.A;
    input_matrix_size = size(input_matrix,2);
    input_matrix_order = colamd(input_matrix);
    R = qr(input_matrix);

    for i = 1:episodes_per_problem
        sample_init_size = randi([round(input_matrix_size/20) round(input_matrix_size/10)], 1);
        random_pick_vars = randsample(input_matrix_size, sample_init_size)';
        
        total_affected_variables = random_pick_vars;
        k = 1;
        
        while k <= length(total_affected_variables)
            row_considered_binary = zeros(1, input_matrix_size);
            row_considered = R(total_affected_variables(k),:);
            row_considered_binary(row_considered ~= 0) = 1;
            assert(~any(total_affected_variables > input_matrix_size));
            for l = total_affected_variables(k)+1:input_matrix_size
                if row_considered_binary(l) > 0 && ~any(total_affected_variables == l)
                    total_affected_variables = [total_affected_variables l];
                end
            end
            k = k + 1;
        end
        
        sample_final_size = length(total_affected_variables);
%         sample_matrix_col_indices = randsample(input_matrix_size, sample_matrix_size);
        sample_matrix_col_indices = total_affected_variables;
        sample_matrix = input_matrix(:, sample_matrix_col_indices);
        sample_matrix_colamd_order = colamd(sample_matrix);

        pos_in_input_matrix = zeros(1, sample_final_size);
        for j = 1:sample_final_size
    %         pos_in_input_matrix(j) = input_matrix_order(sample_matrix_col_indices(j));
            pos_in_input_matrix(j) = find(input_matrix_order == sample_matrix_col_indices(j));
        end

%         [~, mlab_order] = sort(pos_in_input_matrix);
        sample_matrix_myorder = zeros(1, sample_final_size);
%         sample_matrix_myordera = zeros(1, sample_matrix_size);

        for j = 1:sample_final_size
            [~,ind] = min(pos_in_input_matrix);
            pos_in_input_matrix(ind) = inf;
            sample_matrix_myorder(j) = ind;
    %         sample_matrix_myordera(j) = find(sample_matrix_col_indices(ind) == sample_matrix_col_indices);
        end

    %     assert(isequal(sample_matrix_myordera, sample_matrix_myorder));


        nnz_raw(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix))/numel(sample_matrix);
        nnz_colamd(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix(:, sample_matrix_colamd_order)))/numel(sample_matrix);
        nnz_myorder(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix(:, sample_matrix_myorder)))/numel(sample_matrix);

        sizes(((sz-1) * episodes_per_problem) + i) = sample_final_size;
        dist(((sz-1) * episodes_per_problem) + i) = pdist2(sample_matrix_colamd_order, sample_matrix_myorder, 'hamming');
    end
end

figure;
hold on;
plot(sizes, dist, 'r*');

figure;
hold on;
plot(nnz_raw);
plot(nnz_colamd);
plot(nnz_myorder)

figure;
hold on;
stem(nnz_raw, 'filled', 'Marker', 'd', 'LineWidth', 0.75);
stem(nnz_colamd, 'filled', 'Marker', 'd', 'LineWidth', 0.75);
stem(nnz_myorder, 'filled', 'Marker', 'd', 'LineWidth', 0.75);

hold off;




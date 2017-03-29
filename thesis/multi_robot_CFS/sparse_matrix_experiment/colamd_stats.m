function [] = colamd_stats(sample_count, varargin)

nvarargs = length(varargin);
dataset = struct('A', {});

combinations = nchoosek(1:nvarargs, sample_count);
for i = 1:nvarargs
    problem = UFget(varargin{i});
    unit_mat.A = problem.A; 
    dataset = [dataset unit_mat];
end

nnz_reorder = zeros(1,size(combinations,1));
nnz_fused_order = zeros(1,size(combinations,1));
nnz_myorder = zeros(1,size(combinations,1));

for i = 1:size(combinations,1)
    for j = 1:size(combinations,2)-1
        big_matrix_unordered = blkdiag(dataset(combinations(i,j)).A, dataset(combinations(i,j+1)).A);
    end
    
    p = colamd(big_matrix_unordered);
    big_matrix_whole_ordered = big_matrix_unordered(:,p);
    
    for j = 1:size(combinations,2)-1
        p1 = colamd(dataset(combinations(i,j)).A);
        p2 = colamd(dataset(combinations(i,j+1)).A);
        big_matrix_parts_ordered = blkdiag(dataset(combinations(i,j)).A(:,p1), dataset(combinations(i,j+1)).A(:,p2));
    end
    
%     Doing my ordering only for sample_count 2.
    if size(dataset(combinations(i,1)).A, 2) > size(dataset(combinations(i,2)).A, 2)
        min_vec = size(dataset(combinations(i,1)).A, 2) + 1:size(big_matrix_unordered,2);
        max_vec = 1:size(dataset(combinations(i,1)).A, 2);
    else
        min_vec = 1:size(dataset(combinations(i,1)).A, 2);
        max_vec = size(dataset(combinations(i,1)).A, 2) + 1:size(big_matrix_unordered,2);
    end
    gap = floor(length(max_vec)/length(min_vec));
    
    myorder = zeros(1,size(big_matrix_unordered,2));
    for j = 0:length(min_vec)-1
        myorder((j*gap)+(j+1): (j+1)*(gap+1)) = [max_vec((j*gap)+1:(j+1)*gap) min_vec(j+1)];
    end
    if (j+1)*(gap+1)+1 < size(big_matrix_unordered,2)
        myorder((j+1)*(gap+1)+1:end) = (j+1)*(gap+1)+1:length(myorder);
    end
    
    big_matrix_myorder = big_matrix_parts_ordered(:,myorder);
    
    R_reorder = qr(big_matrix_whole_ordered);
    nnz_reorder(i) = nnz(R_reorder);
    R_fused_order = qr(big_matrix_parts_ordered);
    nnz_fused_order(i) = nnz(R_fused_order);
    R_myorder = qr(big_matrix_myorder);
    nnz_myorder(i) = nnz(R_myorder);
    
    big_matrix_whole_ordered = [];
    R_reorder = [];
    big_matrix_parts_ordered = [];
    R_fused_order = [];
    big_matrix_myorder = [];
    R_myorder = [];
    
end

figure; hold on;
plot(nnz_reorder);
plot(nnz_fused_order)
plot(nnz_myorder);

% figure; 
% subplot(1,3,1);
% spy(R_reorder);
% subplot(1,3,2);
% spy(R_fused_order);
% subplot(1,3,3);
% spy(R_myorder);

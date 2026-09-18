#pragma once
#include"standardhead.h"

class Matrix {//缩略该class可以看到矩阵用法说明
private:
    size_t rows_, cols_;
    vector<double> data_;  // 一维数组存储，提高缓存命中率

public:
    // 构造函数（指定维度，可选初始化列表）
    Matrix(size_t rows = 0, size_t cols = 0,
        std::initializer_list<std::initializer_list<double>> init = {})
        : rows_(rows), cols_(cols), data_(rows* cols, 0.0) {

        // 如果有初始化列表，则使用它来初始化
        if (init.size() > 0) {
            if (init.size() != rows_ || init.begin()->size() != cols_) {
                throw std::invalid_argument("初始化列表维度与指定维度不匹配！");
            }

            size_t index = 0;
            for (const auto& row : init) {
                if (row.size() != cols_) {
                    throw std::invalid_argument("所有行的列数必须相同！");
                }
                for (double val : row) {
                    data_[index++] = val;
                }
            }
        }
    }

    // 保留原来的初始化列表构造函数
    Matrix(std::initializer_list<std::initializer_list<double>> init) {
        rows_ = init.size();
        cols_ = (rows_ == 0) ? 0 : init.begin()->size();
        data_.resize(rows_ * cols_);

        size_t index = 0;
        for (const auto& row : init) {
            if (row.size() != cols_) {
                throw std::invalid_argument("所有行的列数必须相同！");
            }
            for (double val : row) {
                data_[index++] = val;
            }
        }
    }
    // 获取行数
    size_t rows() const { return rows_; }

    // 获取列数
    size_t cols() const { return cols_; }

    // 访问元素（可修改）
    double& operator()(size_t row, size_t col) {
        if (row >= rows_ || col >= cols_) {
            throw out_of_range("矩阵索引越界！");
        }
        return data_[row * cols_ + col];
    }
    // 获取内部数据的首地址（可修改版本）
    double* data() {
        return data_.data();  // vector::data() 返回容器内部数组的首地址
    }

    // 获取内部数据的首地址（只读版本，用于const对象）
    const double* data() const {
        return data_.data();
    }
    // 访问元素（只读）
    double operator()(size_t row, size_t col) const {
        if (row >= rows_ || col >= cols_) {
            throw out_of_range("矩阵索引越界！");
        }
        return data_[row * cols_ + col];
    }
    // 生成单位矩阵
    Matrix identity(size_t n) {
        Matrix result(n, n);
        for (size_t i = 0; i < n; ++i) {
            result(i, i) = 1.0;
        }
        return result;
    }
    // 取矩阵的某一行
    Matrix row(size_t row_index) const {
        if (row_index >= rows_) {
            throw out_of_range("行索引越界！");
        }

        Matrix row_vector(1, cols_);  // 创建 1×cols_ 的行向量
        for (size_t j = 0; j < cols_; ++j) {
            row_vector(0, j) = (*this)(row_index, j);  // 复制数据
        }
        return row_vector;
    }
    // 取矩阵的某一列
    Matrix col(size_t col_index) const {
        if (col_index >= cols_) {
            throw out_of_range("列索引越界！");
        }

        Matrix col_vector(rows_, 1);  // 创建 rows_×1 的列向量
        for (size_t i = 0; i < rows_; ++i) {
            col_vector(i, 0) = (*this)(i, col_index);  // 复制数据
        }
        return col_vector;
    }
    // 矩阵加法
    Matrix operator+(const Matrix& other) const {
        if (rows_ != other.rows_ || cols_ != other.cols_) {
            throw invalid_argument("矩阵维度必须相同才能相加！");
        }

        Matrix result(rows_, cols_);
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < cols_; ++j) {
                result(i, j) = (*this)(i, j) + other(i, j);
            }
        }
        return result;
    }
    // 矩阵减法
    Matrix operator-(const Matrix& other) const {
        if (rows_ != other.rows_ || cols_ != other.cols_) {
            throw invalid_argument("矩阵维度必须相同才能相减！");
        }

        Matrix result(rows_, cols_);
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < cols_; ++j) {
                result(i, j) = (*this)(i, j) - other(i, j);
            }
        }
        return result;
    }
    // 矩阵乘法（类似Eigen的 operator*）
    Matrix operator*(const Matrix& other) const {
        if (cols_ != other.rows_) {
            throw invalid_argument("矩阵维度不匹配，无法相乘！");
        }

        Matrix result(rows_, other.cols_);
        for (size_t i = 0; i < rows_; i++) {
            for (size_t j = 0; j < other.cols_; j++) {
                double sum = 0;
                for (size_t k = 0; k < cols_; k++) {
                    sum += (*this)(i, k) * other(k, j);
                }
                result(i, j) = sum;
            }
        }
        return result;
    }
    // 矩阵转置
    Matrix transpose() const {
        Matrix result(cols_, rows_);  // 新矩阵行列互换
        for (size_t i = 0; i < rows_; ++i) {
            for (size_t j = 0; j < cols_; ++j) {
                result(j, i) = (*this)(i, j);  // 交换行列索引
            }
        }
        return result;
    }
    // 矩阵求逆（仅支持方阵）
    Matrix inverse() const {
        if (rows_ != cols_) {
            throw invalid_argument("矩阵必须是方阵才能求逆！");
        }

        size_t n = rows_;
        Matrix augmented(n, 2 * n);  // 创建增广矩阵 [A | I]

        // 初始化增广矩阵
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                augmented(i, j) = (*this)(i, j);  // 左半部分为原矩阵
            }
            augmented(i, i + n) = 1.0;  // 右半部分为单位矩阵
        }

        // 高斯-约旦消元法
        for (size_t i = 0; i < n; ++i) {
            // 寻找主元
            size_t pivot = i;
            for (size_t j = i + 1; j < n; ++j) {
                if (fabs(augmented(j, i)) > fabs(augmented(pivot, i))) {
                    pivot = j;
                }
            }

            // 如果主元为0，矩阵不可逆
            if (fabs(augmented(pivot, i)) < 1e-10) {
                throw runtime_error("矩阵不可逆！");
            }

            // 交换行
            if (pivot != i) {
                for (size_t j = 0; j < 2 * n; ++j) {
                    swap(augmented(i, j), augmented(pivot, j));
                }
            }

            // 归一化主元行
            double divisor = augmented(i, i);
            for (size_t j = 0; j < 2 * n; ++j) {
                augmented(i, j) /= divisor;
            }

            // 消元其他行
            for (size_t k = 0; k < n; ++k) {
                if (k != i) {
                    double factor = augmented(k, i);
                    for (size_t j = 0; j < 2 * n; ++j) {
                        augmented(k, j) -= factor * augmented(i, j);
                    }
                }
            }
        }

        // 提取逆矩阵（增广矩阵的右半部分）
        Matrix result(n, n);
        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j < n; ++j) {
                result(i, j) = augmented(i, j + n);
            }
        }

        return result;
    }

    // 从二维数组创建矩阵
    template<size_t Rows, size_t Cols>
    static Matrix fromArray(const double(&arr)[Rows][Cols]) {
        Matrix result(Rows, Cols);
        for (size_t i = 0; i < Rows; ++i) {
            for (size_t j = 0; j < Cols; ++j) {
                result(i, j) = arr[i][j];
            }
        }
        return result;
    }

    // 从一维数组创建列向量
    template<size_t Size>
    static Matrix fromVector(const double(&arr)[Size]) {
        Matrix result(Size, 1);  // 创建 Size×1 的列向量
        for (size_t i = 0; i < Size; ++i) {
            result(i, 0) = arr[i];
        }
        return result;
    }

    // 新增功能：删除全为0的行，返回新矩阵
    Matrix removeZeroRows() const {
        // 空矩阵直接返回
        if (rows_ == 0 || cols_ == 0) {
            return Matrix();
        }

        // 计算非零行的数量
        size_t nonZeroRows = 0;
        for (size_t i = 0; i < rows_; i++) {
            if (!isRowZero(i)) {
                nonZeroRows++;
            }
        }

        // 如果所有行都是0，返回空矩阵
        if (nonZeroRows == 0) {
            return Matrix();
        }

        // 创建新矩阵存储非零行
        Matrix result(nonZeroRows, cols_);
        size_t newRow = 0;

        for (size_t i = 0; i < rows_; i++) {
            if (!isRowZero(i)) {
                // 复制非零行到新矩阵
                for (size_t j = 0; j < cols_; j++) {
                    result(newRow, j) = (*this)(i, j);
                }
                newRow++;
            }
        }

        return result;
    }
    // 新增功能：删除全为0的列，返回新矩阵
    Matrix removeZeroCols() const {
        // 空矩阵直接返回
        if (rows_ == 0 || cols_ == 0) {
            return Matrix();
        }

        // 计算非零列的数量
        size_t nonZeroCols = 0;
        for (size_t j = 0; j < cols_; j++) {
            if (!isColumnZero(j)) {
                nonZeroCols++;
            }
        }

        // 如果所有列都是0，返回空矩阵
        if (nonZeroCols == 0) {
            return Matrix();
        }

        // 创建新矩阵存储非零列
        Matrix result(rows_, nonZeroCols);
        size_t newCol = 0;

        for (size_t j = 0; j < cols_; j++) {
            if (!isColumnZero(j)) {
                // 复制非零列到新矩阵
                for (size_t i = 0; i < rows_; i++) {
                    result(i, newCol) = (*this)(i, j);
                }
                newCol++;
            }
        }

        return result;
    }
    Matrix removeZeroRowsAndCols() const {
        // 先去除全零行
        Matrix matWithoutZeroRows = removeZeroRows();
        // 再在去除全零行的结果上去除全零列
        return matWithoutZeroRows.removeZeroCols();
    }

private:
    // 辅助函数：检查指定行是否全为0
    bool isRowZero(size_t rowIndex) const {
        if (rowIndex >= rows_) {
            throw out_of_range("行索引越界！");
        }

        // 定义一个小的epsilon值用于浮点数比较
        const double epsilon = 1e-10;

        for (size_t j = 0; j < cols_; j++) {
            if (fabs((*this)(rowIndex, j)) > epsilon) {
                return false;  // 发现非零元素
            }
        }

        return true;  // 所有元素都是0
    }
    // 辅助函数：检查指定列是否全为0
    bool isColumnZero(size_t colIndex) const {
        if (colIndex >= cols_) {
            throw out_of_range("列索引越界！");
        }

        // 定义一个小的epsilon值用于浮点数比较
        const double epsilon = 1e-10;

        for (size_t i = 0; i < rows_; i++) {
            if (fabs((*this)(i, colIndex)) > epsilon) {
                return false;  // 发现非零元素
            }
        }

        return true;  // 所有元素都是0
    }
public:
    // 行视图类（仅支持初始化列表赋值）
    class RowView {
    private:
        Matrix& matrix_;      // 引用原始矩阵
        size_t row_index_;    // 目标行索引

    public:
        RowView(Matrix& matrix, size_t row_index)
            : matrix_(matrix), row_index_(row_index) {}

        // 用初始化列表修改行数据
        void operator=(std::initializer_list<double> list) {
            if (list.size() != matrix_.cols_) {
                throw std::invalid_argument("初始化列表长度必须等于矩阵列数");
            }
            size_t col = 0;
            for (double val : list) {
                matrix_(row_index_, col++) = val;
            }
        }
    };

    // 获取可修改的行视图
    RowView row(size_t row_index) {
        if (row_index >= rows_) {
            throw std::out_of_range("行索引越界！");
        }
        return RowView(*this, row_index);
    }
    // 判断矩阵是否可以求逆
    bool isInvertible() const {
        // 1. 检查是否为方阵
        if (rows_ != cols_) {
            return false;
        }

        // 2. 检查矩阵是否为零矩阵（零矩阵不可逆）
        if (rows_ == 0) {
            return false;
        }

        // 3. 特殊情况：1x1矩阵，判断元素是否为0
        if (rows_ == 1) {
            return fabs(data_[0]) > 1e-10;
        }

        // 4. 通用情况：使用高斯消元法检查秩是否等于阶数
        Matrix temp = *this;  // 复制原矩阵，避免修改原始数据
        size_t rank = 0;

        for (size_t i = 0; i < rows_; ++i) {
            // 寻找主元（当前列中第一个非零元素）
            size_t pivot = i;
            while (pivot < rows_ && fabs(temp(pivot, i)) < 1e-10) {
                pivot++;
            }

            // 如果当前列全为0，跳过（秩不增加）
            if (pivot == rows_) {
                continue;
            }

            // 交换行
            if (pivot != i) {
                for (size_t j = 0; j < cols_; ++j) {
                    swap(temp(i, j), temp(pivot, j));
                }
            }

            // 消元其他行
            for (size_t k = 0; k < rows_; ++k) {
                if (k != i && fabs(temp(k, i)) > 1e-10) {
                    double factor = temp(k, i) / temp(i, i);
                    for (size_t j = i; j < cols_; ++j) {
                        temp(k, j) -= factor * temp(i, j);
                    }
                }
            }

            // 秩加1
            rank++;
        }

        // 矩阵可逆当且仅当秩等于阶数
        return rank == rows_;
    }

    // 将小矩阵嵌入到大矩阵的左上角（左上角对齐），返回新矩阵（原矩阵不变）
    Matrix leftEmbed(const Matrix& smallMat) const {  // 加const确保不修改原矩阵
        // 检查小矩阵是否超过大矩阵尺寸
        if (smallMat.rows() > rows_ || smallMat.cols() > cols_) {
            throw std::out_of_range("Small matrix is too large to embed in top-left corner!");
        }

        // 创建原矩阵的副本（新矩阵，初始值与原矩阵相同）
        Matrix result = *this;

        // 在副本的左上角嵌入小矩阵（覆盖对应位置）
        for (size_t i = 0; i < smallMat.rows(); ++i) {
            for (size_t j = 0; j < smallMat.cols(); ++j) {
                result(i, j) = smallMat(i, j);  // 修改副本，不影响原矩阵
            }
        }

        return result;  // 返回嵌入后的新矩阵
    }

    // 将小矩阵嵌入到大矩阵的右下角（右下角对齐），返回新矩阵（原矩阵不变）
    Matrix rightEmbed(const Matrix& smallMat) const {  // 加const确保不修改原矩阵
        // 计算右下角起始行和列
        size_t startRow = rows_ - smallMat.rows();
        size_t startCol = cols_ - smallMat.cols();

        // 检查小矩阵是否超过大矩阵尺寸
        if (smallMat.rows() > rows_ || smallMat.cols() > cols_) {
            throw std::out_of_range("Small matrix is too large to embed in bottom-right corner!");
        }

        // 创建原矩阵的副本（新矩阵，初始值与原矩阵相同）
        Matrix result = *this;

        // 在副本的右下角嵌入小矩阵（覆盖对应位置）
        for (size_t i = 0; i < smallMat.rows(); ++i) {
            for (size_t j = 0; j < smallMat.cols(); ++j) {
                result(startRow + i, startCol + j) = smallMat(i, j);  // 修改副本，不影响原矩阵
            }
        }

        return result;  // 返回嵌入后的新矩阵
    }
    // 返回动态数组（vector），直接包含全0行索引和大小
    std::vector<size_t> getZeroRowVector() const {
        std::vector<size_t> zeroRows;
        for (size_t row = 0; row < rows_; ++row) {
            if (isRowZero(row)) {
                zeroRows.push_back(row);  // 自动扩容
            }
        }
        return zeroRows;
    }

    // 全0列同理
    std::vector<size_t> getZeroColVector() const {
        std::vector<size_t> zeroCols;
        for (size_t col = 0; col < cols_; ++col) {
            if (isColumnZero(col)) {
                zeroCols.push_back(col);
            }
        }
        return zeroCols;
    }
    // 新增：查询指定行范围内的全0行（[startRow, endRow]，闭区间）
    std::vector<size_t> getZeroRowVectorInRange(size_t startRow, size_t endRow) const {
        std::vector<size_t> zeroRowsInRange;

        // 边界检查：确保startRow <= endRow，且不超过矩阵实际行数
        if (startRow > endRow || endRow >= rows_) {
            throw std::out_of_range("行范围超出矩阵有效行数！");
        }

        // 遍历指定范围内的行，检查是否为全0行
        for (size_t row = startRow; row <= endRow; ++row) {
            if (isRowZero(row)) {
                zeroRowsInRange.push_back(row);  // 存储原始行索引
            }
        }

        return zeroRowsInRange;
    }

    // 新增：查询指定列范围内的全0列（[startCol, endCol]，闭区间）
    std::vector<size_t> getZeroColVectorInRange(size_t startCol, size_t endCol) const {
        std::vector<size_t> zeroColsInRange;

        // 边界检查：确保startCol <= endCol，且不超过矩阵实际列数
        if (startCol > endCol || endCol >= cols_) {
            throw std::out_of_range("列范围超出矩阵有效列数！");
        }

        // 遍历指定范围内的列，检查是否为全0列
        for (size_t col = startCol; col <= endCol; ++col) {
            if (isColumnZero(col)) {
                zeroColsInRange.push_back(col);  // 存储原始列索引
            }
        }

        return zeroColsInRange;
    }
    // 删除指定行（参数：要删除的行索引vector，无unordered_set）
    Matrix removeRows(const std::vector<size_t>& rowIndices) const {
        if (rowIndices.empty()) return *this; // 无要删的行，返回原矩阵

        std::vector<size_t> keep; // 存储需要保留的行索引
        // 遍历原矩阵所有行，判断是否需要保留
        for (size_t i = 0; i < rows_; ++i) {
            bool needDelete = false;
            // 检查当前行是否在删除列表中
            for (size_t delRow : rowIndices) {
                if (i == delRow) {
                    needDelete = true;
                    break; // 找到就退出，不用继续遍历
                }
            }
            if (!needDelete) {
                keep.push_back(i); // 不需要删除，加入保留列表
            }
        }

        // 构造新矩阵
        Matrix res(keep.size(), cols_);
        for (size_t newRow = 0; newRow < keep.size(); ++newRow) {
            size_t oldRow = keep[newRow];
            for (size_t col = 0; col < cols_; ++col) {
                res(newRow, col) = (*this)(oldRow, col);
            }
        }
        return res;
    }

    // 删除指定列（参数：要删除的列索引vector，无unordered_set）
    Matrix removeCols(const std::vector<size_t>& colIndices) const {
        if (colIndices.empty()) return *this; // 无要删的列，返回原矩阵

        std::vector<size_t> keep; // 存储需要保留的列索引
        // 遍历原矩阵所有列，判断是否需要保留
        for (size_t j = 0; j < cols_; ++j) {
            bool needDelete = false;
            // 检查当前列是否在删除列表中
            for (size_t delCol : colIndices) {
                if (j == delCol) {
                    needDelete = true;
                    break; // 找到就退出，不用继续遍历
                }
            }
            if (!needDelete) {
                keep.push_back(j); // 不需要删除，加入保留列表
            }
        }

        // 构造新矩阵
        Matrix res(rows_, keep.size());
        for (size_t newCol = 0; newCol < keep.size(); ++newCol) {
            size_t oldCol = keep[newCol];
            for (size_t row = 0; row < rows_; ++row) {
                res(row, newCol) = (*this)(row, oldCol);
            }
        }
        return res;
    }
    // 返回一个与当前矩阵同阶数的单位矩阵（不修改原矩阵）
    Matrix toIdentity() const {
        // 检查是否为方阵，非方阵无法生成同阶单位矩阵
        if (rows_ != cols_) {
            throw std::invalid_argument("矩阵必须是方阵才能生成同阶单位矩阵！");
        }

        // 创建同阶数的新矩阵（初始全为0）
        Matrix identityMat(rows_, cols_);

        // 对角线元素设为1.0
        for (size_t i = 0; i < rows_; ++i) {
            identityMat(i, i) = 1.0;
        }

        return identityMat;
    }
    // 输出矩阵（支持 cout << matrix）
    friend ostream& operator<<(ostream& os, const Matrix& mat) {
        for (size_t i = 0; i < mat.rows_; i++) {
            for (size_t j = 0; j < mat.cols_; j++) {
                os << mat(i, j) << " ";
            }
            os << "\n";
        }
        return os;
    }
};
/*
矩阵用法说明：
1.构造矩阵
// 默认构造 (全零矩阵)
Matrix mat1(3, 4);  // 3行4列全零矩阵
// 初始化列表构造（二维数组的外维序数是行序，内维序数是列序）
Matrix mat2 = {
    {1, 2, 3},
    {4, 5, 6},
    {7, 8, 9}
};
或者
Matrix c(1, 2, { {0, 1} });;

2.读写元素 (使用operator())
mat2(0, 1) = 10;  // 修改第0行第1列元素
double val = mat2(0, 1);  // 获取第0行第1列元素

// 注意: 索引从0开始，越界会抛出out_of_range异常
3.矩阵行列数
size_t r = mat2.rows();  // 获取行数
size_t c = mat2.cols();  // 获取列数
4.生成单位矩阵
Matrix eye = Matrix().identity(4);  // 4x4单位矩阵
5.提取某一行或一列
// 提取行
Matrix row1 = mat2.row(1);  // 提取第1行(索引从0开始)
// 提取列
Matrix col2 = mat2.col(2);  // 提取第2列(索引从0开始)
6.矩阵运算
// 矩阵加法
Matrix sum = mat1 + mat2;
// 矩阵减法
Matrix diff = mat1 - mat2;
// 矩阵乘法
Matrix prod = mat1 * mat2;
// 矩阵转置
Matrix transposed = mat2.transpose();
// 矩阵求逆 (仅方阵)
Matrix inv = mat2.inverse();
7.将一维数组转为列向量或者二维数组转为矩阵
double arr2D[2][3] = {
        {1, 2, 3},
        {4, 5, 6}
    };
 Matrix mat = Matrix::fromArray(arr2D);

 double arr1D[4] = {1, 2, 3, 4};
Matrix vec = Matrix::fromVector(arr1D);
8.删除全0行、列
mat1.removeZeroRows()；
9.打包左上角右下角
//大.leftembed(小);
//大.rightembed(小);
10.动态数组 获取全0行/列索引
//A.getZeroRowIndices();
//A.getZeroColIndices()；
//带范围的
//A.getZeroRowIndices(0，10086);
//A.getZeroColIndices(1，1008611);
11.删除指定行列
//A=A.removeRows(vector);
//A=A.removeCols(vector);
10.输出矩阵内容
cout << mat;



*/
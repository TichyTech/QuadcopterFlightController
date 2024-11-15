#ifndef MEMORY_storage_H
#define MEMORY_storage_H

namespace BLA {

template<int rows, int cols, class MemT> class Matrix;

///////////////////////////////////////////////////////////////// Array Memory storage ///////////////////////////////////////////////////////////////////

template<int rows, int cols = 1, class ElemT = float> struct Array
{
    typedef ElemT elem_t;
    mutable elem_t m[rows * cols];

    ElemT &operator()(int row, int col) const
    {
        static elem_t dummy;

        if(row > rows || col > cols)
            return dummy;
        else
            return m[row * cols + col];
    }
};

template<int rows, int cols, class ElemT, class opElemT, class retElemT>
Matrix<rows,cols,Array<rows,cols,retElemT> > &Add(const Matrix<rows,cols,Array<rows,cols,ElemT> > &A, const Matrix<rows,cols,Array<rows,cols,opElemT> > &B, Matrix<rows,cols,Array<rows,cols,retElemT> > &C)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            C.storage.m[i * cols + j] = A.storage.m[i * cols + j] + B.storage.m[i * cols + j];

    return C;
}

template<int rows, int cols, class ElemT, class opElemT, class retElemT>
Matrix<rows,cols,Array<rows,cols,retElemT> > &Subtract(const Matrix<rows,cols,Array<rows,cols,ElemT> > &A, const Matrix<rows,cols,Array<rows,cols,opElemT> > &B, Matrix<rows,cols,Array<rows,cols,retElemT> > &C)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            C.storage.m[i * cols + j] = A.storage.m[i * cols + j] - B.storage.m[i * cols + j];

    return C;
}

template<int rows, int cols, int operandCols, class ElemT, class opElemT, class retElemT>
Matrix<rows,operandCols,Array<rows,operandCols,retElemT> > &Multiply(const Matrix<rows,cols,Array<rows,cols,ElemT> > &A, const Matrix<cols,operandCols,Array<cols,operandCols,opElemT> > &B, Matrix<rows,operandCols,Array<rows,operandCols,retElemT> > &C)
{
    int i,j,k;

    for(i = 0; i < rows; i++)
        for(j = 0; j < operandCols; j++)
        {
            if(cols > 0)
                C.storage.m[i * operandCols + j] = A.storage.m[i * cols] * B.storage.m[j];

            for(k = 1; k < cols; k++)
                C.storage.m[i * operandCols + j] += A.storage.m[i * cols + k] * B.storage.m[k * operandCols + j];
        }

    return C;
}

template<int rows, int cols = 1, class ElemT = float> using ArrayMatrix = Matrix<rows,cols,Array<rows,cols,ElemT> >;

///////////////////////////////////////////////////////////////// Reference Memory storage ///////////////////////////////////////////////////////////////////

template<class MemT> struct Reference
{
    typedef typename MemT::elem_t elem_t;

    const MemT &parent;
    int rowOffset, colOffset;

    Reference<MemT>(const MemT &obj, int rowOff, int colOff) : parent(obj), rowOffset(rowOff), colOffset(colOff) { }
    Reference<MemT>(const Reference<MemT> &obj) : parent(obj.parent), rowOffset(obj.rowOffset), colOffset(obj.colOffset) { }

    typename MemT::elem_t &operator()(int row, int col) const
    {
        return parent(row+rowOffset, col+colOffset);
    }
};

template<int rows, int cols, class ElemT = float> using ArrayRef = Reference<Array<rows,cols,ElemT> >;
template<int rows, int cols, class ParentMemT > using RefMatrix = Matrix<rows,cols,Reference<ParentMemT> >;


///////////////////////////////////////////////////////////////// Identity Memory storage ///////////////////////////////////////////////////////////////////

template<class ElemT> struct Iden
{
    typedef ElemT elem_t;

    elem_t &operator()(int row, int col) const
    {
        static elem_t ret;

        if(row == col)
            return (ret = 1);
        else
            return (ret = 0);
    }
};

template<int rows, int cols = rows, class ElemT = float> using Identity = Matrix<rows,cols,Iden<ElemT> >;

///////////////////////////////////////////////////////////////// Zeros Memory storage ///////////////////////////////////////////////////////////////////

template<class ElemT> struct Zero
{
    typedef ElemT elem_t;

    ElemT &operator()(int row, int col) const
    {
        static ElemT ret;

        return (ret = 0);
    }
};

template<int rows, int cols = 1, class ElemT = float> using Zeros = Matrix<rows,cols,Zero<ElemT> >;


///////////////////////////////////////////////////////////////// Sparse Memory storage ///////////////////////////////////////////////////////////////////

// This uses a hash table to look up row/col/val items. It uses an open addressing collision strategy so we can avoid using dynamic memory
template<int cols, int tableSize, class ElemT> struct Sparse
{
    typedef ElemT elem_t;

    struct HashItem
    {
        mutable int key;
        mutable ElemT val;

        HashItem() { key = -1; }

    } table[tableSize];

    ElemT &operator()(int row, int col) const
    {
        // Make a key out of the row / column
        int key = row * cols + col;

        // Calculate the hash by taking the modulo of the key with the tableSize
        int hash = key % tableSize;

        const HashItem *item;

        // Find a item with a key matching the input key
        for(int i = 0; i < tableSize; i++)
        {
            item = table + (hash + i) % tableSize;

            // If the element is empty or unused (val == 0) then the item doesn't exist in the table
            if(item->key == -1 || item->val == 0)
            {
                item->key = key;
                item->val = 0;
                break;
            }

            // If it's key matches the input key then return it
            if(item->key == key)
            {
                break;
            }
        }

        // If we landed on a matching key then we're done!
        if(item->key == key)
        {
            return item->val;
        }
        else
        {
            static ElemT outOfMemory;
            return outOfMemory;
        }
    }
};

template<int rows, int cols, int tableSize = cols, class ElemT = float> using SparseMatrix = Matrix<rows,cols,Sparse<cols,tableSize,ElemT> >;

//////////////////////////////////////////////////////////// Matrix Minor Memory storage ////////////////////////////////////////////////////////////////

template<class MemT> struct Minor
{
    typedef typename MemT::elem_t elem_t;
    const MemT parent;
    int i, j;

    Minor<MemT>(const MemT &obj, int row, int col) : parent(obj), i(row), j(col) { }

    elem_t &operator()(int row, int col) const
    {
        if(row >= i) row++;
        if(col >= j) col++;

        return parent(row,col);
    }
};

//////////////////////////////////////////////////////////// Transpose storage ////////////////////////////////////////////////////////////////

template<class MemT> struct Trans
{
    typedef typename MemT::elem_t elem_t;
    const MemT parent;

    Trans<MemT>(const MemT &obj) : parent(obj) { }
    Trans<MemT>(const Trans<MemT> &obj) : parent(obj.parent) { }

    elem_t &operator()(int row, int col) const
    {
        return parent(col,row);
    }
};

////////////////////////////////////////////////////////// Concatenation storages /////////////////////////////////////////////////////////////

template<int leftCols, class LeftMemT, class RightMemT> struct HorzCat
{
    typedef typename LeftMemT::elem_t elem_t;
    const LeftMemT left;
    const RightMemT right;

    HorzCat<leftCols,LeftMemT,RightMemT>(const LeftMemT &l, const RightMemT &r) : left(l), right(r) { }
    HorzCat<leftCols,LeftMemT,RightMemT>(const HorzCat<leftCols,LeftMemT,RightMemT> &obj) : left(obj.left), right(obj.right) { }

    virtual ~HorzCat<leftCols,LeftMemT,RightMemT>() { }

    elem_t &operator()(int row, int col) const
    {
        return col < leftCols? left(row,col) : right(row,col-leftCols);
    }
};

template<int topRows, class TopMemT, class BottomMemT> struct VertCat
{
    typedef typename TopMemT::elem_t elem_t;
    const TopMemT top;
    const BottomMemT bottom;

    VertCat<topRows,TopMemT,BottomMemT>(const TopMemT &t, const BottomMemT &b) : top(t), bottom(b) { }
    VertCat<topRows,TopMemT,BottomMemT>(const VertCat<topRows,TopMemT,BottomMemT> &obj) : top(obj.top), bottom(obj.bottom) { }

    virtual ~VertCat<topRows,TopMemT,BottomMemT>() { }

    elem_t &operator()(int row, int col) const
    {
        return row < topRows? top(row,col) : bottom(row-topRows,col);
    }
};

} // namespace BLA

#endif // MEMORY_storage_H

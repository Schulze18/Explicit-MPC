/*****************************************************************************/
/* Name: matrix.h                                                            */
/* Uses: Class for matrix math functions.                                    */
/* Date: 4/19/2011                                                           */
/* Author: Andrew Que <http://www.DrQue.net/>                                */
/* Revisions:                                                                */
/*   0.1 - 2011/04/19 - QUE - Creation.                                      */
/*   0.5 - 2011/04/24 - QUE - Most functions are complete.                   */
/*   0.8 - 2011/05/01 - QUE -                                                */
/*     = Bug fixes.                                                          */
/*     + Dot product.                                                        */
/*   1.0 - 2011/11/26 - QUE - Release.                                       */
/*                                                                           */
/* Notes:                                                                    */
/*   This unit implements some very basic matrix functions, which include:   */
/*    + Addition/subtraction                                                 */
/*    + Transpose                                                            */
/*    + Row echelon reduction                                                */
/*    + Determinant                                                          */
/*    + Dot product                                                          */
/*    + Matrix product                                                       */
/*    + Scalar product                                                       */
/*    + Inversion                                                            */
/*    + LU factorization/decomposition                                       */
/*     There isn't much for optimization in this unit as it was designed as  */
/*   more of a learning experience.                                          */
/*                                                                           */
/* License:                                                                  */
/*   This program is free software: you can redistribute it and/or modify    */
/*   it under the terms of the GNU General Public License as published by    */
/*   the Free Software Foundation, either version 3 of the License, or       */
/*   (at your option) any later version.                                     */
/*                                                                           */
/*   This program is distributed in the hope that it will be useful,         */
/*   but WITHOUT ANY WARRANTY; without even the implied warranty of          */
/*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           */
/*   GNU General Public License for more details.                            */
/*                                                                           */
/*   You should have received a copy of the GNU General Public License       */
/*   along with this program.  If not, see <http://www.gnu.org/licenses/>.   */
/*                                                                           */
/*                     (C) Copyright 2011 by Andrew Que                      */
/*                           http://www.DrQue.net/                           */
/*****************************************************************************/
#ifndef _MATRIX_DRQUE_H_ //name of the library changed to avoid ambiguity with matlab library
#define _MATRIX_DRQUE_H_

#include <iostream>
#include <cassert>
#include <climits>
#include <vector>

// Class forward for identity matrix.
template< class TYPE > class IdentityMatrix;

//=============================================================================
// Matrix template class
//   Contains a set of matrix manipulation functions.  The template is designed
// so that the values of the matrix can be of any type that allows basic
// arithmetic.
//=============================================================================
template< class TYPE = int >
  class Matrix
  {
    protected:
      // Matrix data.
      unsigned rows;
      unsigned columns;

      // Storage for matrix data.
      std::vector< std::vector< TYPE > > matrix;

      // Order sub-index for rows.
      //   Use: matrix[ order[ row ] ][ column ].
      unsigned * order;

      //-------------------------------------------------------------
      // Return the number of leading zeros in the given row.
      //-------------------------------------------------------------
      unsigned getLeadingZeros
      (
        // Row to count
        unsigned row
      ) const
      {
        TYPE const ZERO = static_cast< TYPE >( 0 );
        unsigned column = 0;
        while ( ZERO == matrix[ row ][ column ] )
          ++column;

        return column;
      }

      //-------------------------------------------------------------
      // Reorder the matrix so the rows with the most zeros are at
      // the end, and those with the least at the beginning.
      //
      // NOTE: The matrix data itself is not manipulated, just the
      // 'order' sub-indexes.
      //-------------------------------------------------------------
      void reorder()
      {
        unsigned * zeros = new unsigned[ rows ];

        for ( unsigned row = 0; row < rows; ++row )
        {
          order[ row ] = row;
          zeros[ row ] = getLeadingZeros( row );
        }

        for ( unsigned row = 0; row < (rows-1); ++row )
        {
          unsigned swapRow = row;
          for ( unsigned subRow = row + 1; subRow < rows; ++subRow )
          {
            if ( zeros[ order[ subRow ] ] < zeros[ order[ swapRow ] ] )
              swapRow = subRow;
          }

          unsigned hold    = order[ row ];
          order[ row ]     = order[ swapRow ];
          order[ swapRow ] = hold;
        }

        delete zeros;
      }

      //-------------------------------------------------------------
      // Divide a row by given value.  An elementary row operation.
      //-------------------------------------------------------------
      void divideRow
      (
        // Row to divide.
        unsigned row,

        // Divisor.
        TYPE const & divisor
      )
      {
        for ( unsigned column = 0; column < columns; ++column )
          matrix[ row ][ column ] /= divisor;
      }

      //-------------------------------------------------------------
      // Modify a row by adding a scaled row. An elementary row
      // operation.
      //-------------------------------------------------------------
      void rowOperation
      (
        unsigned row,
        unsigned addRow,
        TYPE const & scale
      )
      {
        for ( unsigned column = 0; column < columns; ++column )
          matrix[ row ][ column ] += matrix[ addRow ][ column ] * scale;
      }

      //-------------------------------------------------------------
      // Allocate memory for matrix data.
      //-------------------------------------------------------------
      void allocate
      (
        unsigned rowNumber,
        unsigned columnNumber
      )
      {
        // Allocate order integers.
        order = new unsigned[ rowNumber ];

        // Setup matrix sizes.
        matrix.resize( rowNumber );
        for ( unsigned row = 0; row < rowNumber; ++row )
          matrix[ row ].resize( columnNumber );
      }

      //-------------------------------------------------------------
      // Free memory used for matrix data.
      //-------------------------------------------------------------
      void deallocate
      (
        unsigned rowNumber,
        unsigned columnNumber
      )
      {
        // Free memory used for storing order (if there is any).
        if ( 0 != rowNumber )
          delete[] order;
      }

    public:
      // Used for matrix concatenation.
      typedef enum
      {
        TO_RIGHT,
        TO_BOTTOM
      } Position;

      //-------------------------------------------------------------
      // Return the number of rows in this matrix.
      //-------------------------------------------------------------
      unsigned getRows() const
      {
        return rows;
      }

      //-------------------------------------------------------------
      // Return the number of columns in this matrix.
      //-------------------------------------------------------------
      unsigned getColumns() const
      {
        return columns;
      }

      //-------------------------------------------------------------
      // Get an element of the matrix.
      //-------------------------------------------------------------
      TYPE get
      (
        unsigned row,   // Which row.
        unsigned column // Which column.
      ) const
      {
        assert( row < rows );
        assert( column < columns );

        return matrix[ row ][ column ];
      }

      //-------------------------------------------------------------
      // Proform LU decomposition.
      // This will create matrices L and U such that A=LxU
      //-------------------------------------------------------------
      void LU_Decomposition
      (
        Matrix & upper,
        Matrix & lower
      ) const
      {
        assert( rows == columns );

        TYPE const ZERO = static_cast< TYPE >( 0 );

        upper = *this;
        lower = *this;

        for ( unsigned row = 0; row < rows; ++row )
          for ( unsigned column = 0; column < columns; ++column )
            lower.matrix[ row ][ column ] = ZERO;

        for ( unsigned row = 0; row < rows; ++row )
        {
          TYPE value = upper.matrix[ row ][ row ];
          if ( ZERO != value )
          {
            upper.divideRow( row, value );
            lower.matrix[ row ][ row ] = value;
          }

          for ( unsigned subRow = row + 1; subRow < rows; ++subRow )
          {
            TYPE value = upper.matrix[ subRow ][ row ];
            upper.rowOperation( subRow, row, -value );
            lower.matrix[ subRow ][ row ] = value;
          }
        }
      }

      //-------------------------------------------------------------
      // Set an element in the matrix.
      //-------------------------------------------------------------
      void put
      (
        unsigned row,
        unsigned column,
        TYPE const & value
      )
      {
        assert( row < rows );
        assert( column < columns );

        matrix[ row ][ column ] = value;
      }

      //-------------------------------------------------------------
      // Return part of the matrix.
      // NOTE: The end points are the last elements copied.  They can
      // be equal to the first element when wanting just a single row
      // or column.  However, the span of the total matrix is
      // ( 0, rows - 1, 0, columns - 1 ).
      //-------------------------------------------------------------
      Matrix getSubMatrix
      (
        unsigned startRow,
        unsigned endRow,
        unsigned startColumn,
        unsigned endColumn,
        unsigned const * newOrder = NULL
      )
      {
        Matrix subMatrix( endRow - startRow + 1, endColumn - startColumn + 1 );

        for ( unsigned row = startRow; row <= endRow; ++row )
        {
          unsigned subRow;
          if ( NULL == newOrder )
            subRow = row;
          else
            subRow = newOrder[ row ];

          for ( unsigned column = startColumn; column <= endColumn; ++column )
            subMatrix.matrix[ row - startRow ][ column - startColumn ] =
              matrix[ subRow ][ column ];
        }

        return subMatrix;
      }

      //-------------------------------------------------------------
      // Return a single column from the matrix.
      //-------------------------------------------------------------
      Matrix getColumn
      (
        unsigned column
      )
      {
        return getSubMatrix( 0, rows - 1, column, column );
      }

      //-------------------------------------------------------------
      // Return a single row from the matrix.
      //-------------------------------------------------------------
      Matrix getRow
      (
        unsigned row
      )
      {
        return getSubMatrix( row, row, 0, columns - 1 );
      }

      //-------------------------------------------------------------
      // Place matrix in reduced row echelon form.
      //-------------------------------------------------------------
      void reducedRowEcholon()
      {
        TYPE const ZERO = static_cast< TYPE >( 0 );

        // For each row...
        for ( unsigned rowIndex = 0; rowIndex < rows; ++rowIndex )
        {
          // Reorder the rows.
          reorder();

          unsigned row = order[ rowIndex ];

          // Divide row down so first term is 1.
          unsigned column = getLeadingZeros( row );
          TYPE divisor = matrix[ row ][ column ];
          if ( ZERO != divisor )
          {
            divideRow( row, divisor );

            // Subtract this row from all subsequent rows.
            for ( unsigned subRowIndex = ( rowIndex + 1 ); subRowIndex < rows; ++subRowIndex )
            {
              unsigned subRow = order[ subRowIndex ];
              if ( ZERO != matrix[ subRow ][ column ] )
                rowOperation
                (
                  subRow,
                  row,
                  -matrix[ subRow ][ column ]
                );
            }
          }

        }

        // Back substitute all lower rows.
        for ( unsigned rowIndex = ( rows - 1 ); rowIndex > 0; --rowIndex )
        {
          unsigned row = order[ rowIndex ];
          unsigned column = getLeadingZeros( row );
          for ( unsigned subRowIndex = 0; subRowIndex < rowIndex; ++subRowIndex )
          {
            unsigned subRow = order[ subRowIndex ];
            rowOperation
            (
              subRow,
              row,
              -matrix[ subRow ][ column ]
            );
          }
        }

      } // reducedRowEcholon

      //-------------------------------------------------------------
      // Return the determinant of the matrix.
      // Recursive function.
      //-------------------------------------------------------------
      TYPE determinant() const
      {
        TYPE result = static_cast< TYPE >( 0 );

        // Must have a square matrix to even bother.
        assert( rows == columns );

        if ( rows > 2 )
        {
          int sign = 1;
          for ( unsigned column = 0; column < columns; ++column )
          {
            TYPE subDeterminant;

            Matrix subMatrix = Matrix( *this, 0, column );

            subDeterminant  = subMatrix.determinant();
            subDeterminant *= matrix[ 0 ][ column ];

            if ( sign > 0 )
              result += subDeterminant;
            else
              result -= subDeterminant;

            sign = -sign;
          }
        }
        else
        {
          result = ( matrix[ 0 ][ 0 ] * matrix[ 1 ][ 1 ] )
                 - ( matrix[ 0 ][ 1 ] * matrix[ 1 ][ 0 ] );
        }

        return result;

      } // determinant

      //-------------------------------------------------------------
      // Calculate a dot product between this and an other matrix.
      //-------------------------------------------------------------
      TYPE dotProduct
      (
        Matrix const & otherMatrix
      ) const
      {
        // Dimentions of each matrix must be the same.
        assert( rows == otherMatrix.rows );
        assert( columns == otherMatrix.columns );

        TYPE result = static_cast< TYPE >( 0 );
        for ( unsigned row = 0; row < rows; ++row )
          for ( unsigned column = 0; column < columns; ++column )
          {
            result +=
              matrix[ row ][ column ]
              * otherMatrix.matrix[ row ][ column ];
          }

        return result;

      } // dotProduct

      //-------------------------------------------------------------
      // Return the transpose of the matrix.
      //-------------------------------------------------------------
      Matrix const getTranspose() const
      {
        Matrix result( columns, rows );

        // Transpose the matrix by filling the result's rows will
        // these columns, and vica versa.
        for ( unsigned row = 0; row < rows; ++row )
          for ( unsigned column = 0; column < columns; ++column )
            result.matrix[ column ][ row ] = matrix[ row ][ column ];

        return result;

      } // transpose

      //-------------------------------------------------------------
      // Transpose the matrix.
      //-------------------------------------------------------------
      void transpose()
      {
        *this = getTranspose();
      }

      //-------------------------------------------------------------
      // Return inverse matrix.
      //-------------------------------------------------------------
      Matrix const getInverse() const
      {
        // Concatenate the identity matrix onto this matrix.
        Matrix inverseMatrix
          (
            *this,
            IdentityMatrix< TYPE >( rows, columns ),
            TO_RIGHT
          );

        // Row reduce this matrix.  This will result in the identity
        // matrix on the left, and the inverse matrix on the right.
        inverseMatrix.reducedRowEcholon();

        // Copy the inverse matrix data back to this matrix.
        Matrix result
        (
          inverseMatrix.getSubMatrix
          (
            0,
            rows - 1,
            columns,
            columns + columns - 1,
            inverseMatrix.order
          )
        );

        return result;

      } // invert


      //-------------------------------------------------------------
      // Invert this matrix.
      //-------------------------------------------------------------
      void invert()
      {
        *this = getInverse();

      } // invert

      //=======================================================================
      // Operators.
      //=======================================================================

      //-------------------------------------------------------------
      // Add by an other matrix.
      //-------------------------------------------------------------
      Matrix const operator +
      (
        Matrix const & otherMatrix
      ) const
      {
        assert( otherMatrix.rows == rows );
        assert( otherMatrix.columns == columns );

        Matrix result( rows, columns );

        for ( unsigned row = 0; row < rows; ++row )
          for ( unsigned column = 0; column < columns; ++column )
            result.matrix[ row ][ column ] =
              matrix[ row ][ column ]
              + otherMatrix.matrix[ row ][ column ];

        return result;
      }

      //-------------------------------------------------------------
      // Add self by an other matrix.
      //-------------------------------------------------------------
      Matrix const & operator +=
      (
        Matrix const & otherMatrix
      )
      {
        *this = *this + otherMatrix;
        return *this;
      }

      //-------------------------------------------------------------
      // Subtract by an other matrix.
      //-------------------------------------------------------------
      Matrix const operator -
      (
        Matrix const & otherMatrix
      ) const
      {
        assert( otherMatrix.rows == rows );
        assert( otherMatrix.columns == columns );

        Matrix result( rows, columns );

        for ( unsigned row = 0; row < rows; ++row )
          for ( unsigned column = 0; column < columns; ++column )
            result.matrix[ row ][ column ] =
              matrix[ row ][ column ]
              - otherMatrix.matrix[ row ][ column ];

        return result;
      }

      //-------------------------------------------------------------
      // Subtract self by an other matrix.
      //-------------------------------------------------------------
      Matrix const & operator -=
      (
        Matrix const & otherMatrix
      )
      {
        *this = *this - otherMatrix;
        return *this;
      }

      //-------------------------------------------------------------
      // Matrix multiplication.
      //-------------------------------------------------------------
      Matrix const operator *
      (
        Matrix const & otherMatrix
      ) const
      {
        TYPE const ZERO = static_cast< TYPE >( 0 );

        assert( otherMatrix.rows == columns );

        Matrix result( rows, otherMatrix.columns );

        for ( unsigned row = 0; row < rows; ++row )
          for ( unsigned column = 0; column < otherMatrix.columns; ++column )
          {
            result.matrix[ row ][ column ] = ZERO;

            for ( unsigned index = 0; index < columns; ++index )
              result.matrix[ row ][ column ] +=
                matrix[ row ][ index ]
                * otherMatrix.matrix[ index ][ column ];
          }

        return result;
      }

      //-------------------------------------------------------------
      // Multiply self by matrix.
      //-------------------------------------------------------------
      Matrix const & operator *=
      (
        Matrix const & otherMatrix
      )
      {
        *this = *this * otherMatrix;
        return *this;
      }

      //-------------------------------------------------------------
      // Multiply by scalar constant.
      //-------------------------------------------------------------
      Matrix const operator *
      (
        TYPE const & scalar
      ) const
      {
        Matrix result( rows, columns );

        for ( unsigned row = 0; row < rows; ++row )
          for ( unsigned column = 0; column < columns; ++column )
            result.matrix[ row ][ column ] = matrix[ row ][ column ] * scalar;

        return result;
      }

      //-------------------------------------------------------------
      // Multiply self by scalar constant.
      //-------------------------------------------------------------
      Matrix const & operator *=
      (
        TYPE const & scalar
      )
      {
        *this = *this * scalar;
        return *this;
      }

      //-------------------------------------------------------------
      // Copy matrix.
      //-------------------------------------------------------------
      Matrix & operator =
      (
        Matrix const & otherMatrix
      )
      {
        if ( this == &otherMatrix )
          return *this;

        // Release memory currently in use.
        deallocate( rows, columns );

        rows    = otherMatrix.rows;
        columns = otherMatrix.columns;
        allocate( rows, columns );

        for ( unsigned row = 0; row < rows; ++row )
          for ( unsigned column = 0; column < columns; ++column )
            matrix[ row ][ column ] =
            otherMatrix.matrix[ row ][ column ];

        return *this;
      }

      //-------------------------------------------------------------
      // Copy matrix data from array.
      // Although matrix data is two dimensional, this copy function
      // assumes the previous row is immediately followed by the next
      // row's data.
      //
      // Example for 3x2 matrix:
      //     int const data[ 3 * 2 ] =
      //     {
      //       1, 2, 3,
      //       4, 5, 6
      //     };
      //    Matrix< int > matrix( 3, 2 );
      //    matrix = data;
      //-------------------------------------------------------------
      Matrix & operator =
      (
        TYPE const * data
      )
      {
        unsigned index = 0;

        for ( unsigned row = 0; row < rows; ++row )
          for ( unsigned column = 0; column < columns; ++column )
            matrix[ row ][ column ] = data[ index++ ];

        return *this;
      }

      //-----------------------------------------------------------------------
      // Return true if this matrix is the same of parameter.
      //-----------------------------------------------------------------------
      bool operator ==
      (
        Matrix const & value
      ) const
      {
        bool isEqual = true;
        for ( unsigned row = 0; row < rows; ++row )
          for ( unsigned column = 0; column < columns; ++column )
            if ( matrix[ row ][ column ] != value.matrix[ row ][ column ] )
              isEqual = false;

        return isEqual;
      }

      //-----------------------------------------------------------------------
      // Return true if this matrix is NOT the same of parameter.
      //-----------------------------------------------------------------------
      bool operator !=
      (
        Matrix const & value
      ) const
      {
        return !( *this == value );
      }

      //-------------------------------------------------------------
      // Constructor for empty matrix.
      // Only useful if matrix is being assigned (i.e. copied) from
      // somewhere else sometime after construction.
      //-------------------------------------------------------------
      Matrix()
      :
        rows( 0 ),
        columns( 0 )
      {
        allocate( 0, 0 );
      }

      //-------------------------------------------------------------
      // Constructor using rows and columns.
      //-------------------------------------------------------------
      Matrix
      (
        unsigned rowsParameter,
        unsigned columnsParameter
      )
      :
        rows( rowsParameter ),
        columns( columnsParameter )
      {
        TYPE const ZERO = static_cast< TYPE >( 0 );

        // Allocate memory for new matrix.
        allocate( rows, columns );

        // Fill matrix with zero.
        for ( unsigned row = 0; row < rows; ++row )
        {
          order[ row ] = row;

          for ( unsigned column = 0; column < columns; ++column )
            matrix[ row ][ column ] = ZERO;
        }
      }

      //-------------------------------------------------------------
      // This constructor will allow the creation of a matrix based off
      // an other matrix.  It can copy the matrix entirely, or omitted a
      // row/column.
      //-------------------------------------------------------------
      Matrix
      (
        Matrix const & copyMatrix,
        unsigned omittedRow    = INT_MAX,
        unsigned omittedColumn = INT_MAX
      )
      {
        // Start with the number of rows/columns from matrix to be copied.
        rows    = copyMatrix.getRows();
        columns = copyMatrix.getColumns();

        // If a row is omitted, then there is one less row.
        if ( INT_MAX != omittedRow  )
          rows--;

        // If a column is omitted, then there is one less column.
        if ( INT_MAX != omittedColumn )
          columns--;

        // Allocate memory for new matrix.
        allocate( rows, columns );

        unsigned rowIndex = 0;
        for ( unsigned row = 0; row < rows; ++row )
        {
          // If this row is to be skipped...
          if ( rowIndex == omittedRow )
            rowIndex++;

          // Set default order.
          order[ row ] = row;

          unsigned columnIndex = 0;
          for ( unsigned column = 0; column < columns; ++column )
          {
            // If this column is to be skipped...
            if ( columnIndex == omittedColumn )
              columnIndex++;

            matrix[ row ][ column ] = copyMatrix.matrix[ rowIndex ][ columnIndex ];

            columnIndex++;
          }

          ++rowIndex;
        }

      }

      //-------------------------------------------------------------
      // Constructor to concatenate two matrices.  Concatenation
      // can be done to the right, or to the bottom.
      //   A = [B | C]
      //-------------------------------------------------------------
      Matrix
      (
        Matrix const & copyMatrixA,
        Matrix const & copyMatrixB,
        Position position = TO_RIGHT
      )
      {
        unsigned rowOffset    = 0;
        unsigned columnOffset = 0;

        if ( TO_RIGHT == position )
          columnOffset = copyMatrixA.columns;
        else
          rowOffset = copyMatrixA.rows;

        rows    = copyMatrixA.rows    + rowOffset;
        columns = copyMatrixA.columns + columnOffset;

        // Allocate memory for new matrix.
        allocate( rows, columns );

        for ( unsigned row = 0; row < copyMatrixA.rows; ++row )
          for ( unsigned column = 0; column < copyMatrixA.columns; ++column )
            matrix[ row ][ column ] = copyMatrixA.matrix[ row ][ column ];

        for ( unsigned row = 0; row < copyMatrixB.rows; ++row )
          for ( unsigned column = 0; column < copyMatrixB.columns; ++column )
            matrix[ row + rowOffset ][ column + columnOffset ] =
              copyMatrixB.matrix[ row ][ column ];
      }

      //-------------------------------------------------------------
      // Destructor.
      //-------------------------------------------------------------
      ~Matrix()
      {
        // Release memory.
        deallocate( rows, columns );
      }

  };

//=============================================================================
// Class for identity matrix.
//=============================================================================
template< class TYPE >
  class IdentityMatrix : public Matrix< TYPE >
  {
    public:
      IdentityMatrix
      (
        unsigned rowsParameter,
        unsigned columnsParameter
      )
      :
        Matrix< TYPE >( rowsParameter, columnsParameter )
      {
        TYPE const ZERO = static_cast< TYPE >( 0 );
        TYPE const ONE  = static_cast< TYPE >( 1 );

        for ( unsigned row = 0; row < Matrix< TYPE >::rows; ++row )
        {
          for ( unsigned column = 0; column < Matrix< TYPE >::columns; ++column )
            if ( row == column )
              Matrix< TYPE >::matrix[ row ][ column ] = ONE;
            else
              Matrix< TYPE >::matrix[ row ][ column ] = ZERO;
        }
      }
  };

//-----------------------------------------------------------------------------
// Stream operator used to convert matrix class to a string.
//-----------------------------------------------------------------------------
template< class TYPE >
  std::ostream & operator<<
  (
    // Stream data to place string.
    std::ostream & stream,

    // A matrix.
    Matrix< TYPE > const & matrix
  )
  {
    for ( unsigned row = 0; row < matrix.getRows(); ++row )
    {
      for ( unsigned column = 0; column < matrix.getColumns(); ++column )
        stream << "\t" << matrix.get( row , column );

      stream << std::endl;
    }

    return stream;
  }

#endif // _MATRIX_H_

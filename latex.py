import math

"""def countingSort(A,B,k):
    C = [0 for _ in range(k)]

    for j in range(0, len(A)):
        C[A[j]] = C[A[j]] + 1

    # print the state of C here.
    print("\\item $C[i]$ contains the number of elements equal to i. \\\\")
    print(f"\t$C = \\langle {str(C).replace('[', '').replace(']', '')} \\rangle$ \\\\")

    for i in range(1, k):
        C[i] = C[i] + C[i-1]

    # print the state of C here.
    print("\\item $C[i]$ contains the number of elements less than or equal to i. \\\\")
    print(f"\t$C = \\langle {str(C).replace('[', '').replace(']', '')} \\rangle$ \\\\")

    for j in range(len(A)-1, -1, -1):
        B[C[A[j]]-1] = A[j]
        C[A[j]] = C[A[j]] - 1
        # print the state of B and C here.
        print(f"\\item iteration ${j}$. \\\\ \n")
        print(f"\t$C = \\langle {str(C).replace('[', '').replace(']', '')} \\rangle$ \\\\")
        print(f"\t$B = \\langle {str(B).replace('[', '').replace(']', '')} \\rangle$ \\\\")

    return B


print("\\begin{enumerate}\n")
A = [6, 0, 2, 0, 1, 3, 4, 6, 1, 3, 2]
C = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
C = countingSort(A, C, 7)
print(f"\t\\item End Result: $\\langle {str(C).replace('[', '').replace(']', '')} \\rangle$ \\\\")
print("\\end{enumerate}\n")"""

def insertionSort(arr):
    # Traverse through 1 to len(arr)
    for i in range(1, len(arr)):
        key = arr[i]
        j = i-1
        while j >=0 and key < arr[j] :
                arr[j+1] = arr[j]
                j -= 1
        arr[j+1] = key

def bucketSort(A):
    n = len(A)
    B = [[] for _ in range(n)]
    
    for i in range(n):
        index = math.floor(n * A[i])
        B[index].append(A[i])

    C = []

    for i in range(n):
        insertionSort(B[i])
        C.extend(B[i])

    print(f"\t\\item B: \\\\")
    for i in range(n):
        print(f"${i} \\rightarrow \\langle {str(B[i]).replace('[', '').replace(']', '')} \\rangle$ \\\\")

    return C



print("\\begin{enumerate}\n")
A = [0.79, 0.13, 0.16, 0.64, 0.39, 0.20, 0.89, 0.53, 0.71, 0.42]
print(f"\t\\item Starting Array: $\\langle {str(A).replace('[', '').replace(']', '')} \\rangle$ \\\\")
C = bucketSort(A)
print(f"\t\\item End Result: $\\langle {str(C).replace('[', '').replace(']', '')} \\rangle$ \\\\")
print("\\end{enumerate}\n")

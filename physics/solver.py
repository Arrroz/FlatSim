import numpy as np

class Solver():

    def __init__(self, max_iterations=1e5, tol=1e-9, debug=False):
        self.M = np.zeros((0,0))
        self.q = np.zeros((0,))
        self.eq_mask = np.zeros((0,), dtype=bool) # marks which rows/cols of M are equality rows, in any order

        self.tol = tol # tolerance for near-zero comparisons in the solver
        self.max_iterations = max_iterations
        self.debug = debug
        self.iteration = 0

    def solve(self):
        pass


class LemkeSolver(Solver):

    def solve(self):
        self.iteration = 0
        return self.lemke4mixed()

    def lemke_pivot(self, entering_i, dropping_i, y_ids, B, c):
        new_B = np.copy(B)
        new_B[:, dropping_i] = c

        new_y_ids = np.copy(y_ids)
        new_y_ids[dropping_i] = entering_i

        return (new_y_ids, new_B)

    def lemke_update_inverse(self, B_inv, B, dropping_i, c): # Sherman-Morrison-Woodbury update (thesis 4.8.1): O(n^2) instead of recomputing pinv(B) at O(n^3) every pivot
        delta_v = c - B[:, dropping_i]
        b_row = B_inv[dropping_i, :]
        u = B_inv @ delta_v
        denom = 1 + b_row @ delta_v
        return B_inv - np.outer(u, b_row) / denom


    def lemke_min_ratio(self, Bc, Bq, B_inv): # returns the row to drop, chosen by the lexico-minimum ratio test
        # a basic variable can only reach zero (and drop out) if its Bc entry is positive; a non-positive
        # entry means that variable grows or stays put as the entering one increases, so it never binds
        dropping_candidates = [i for i in range(self.n) if Bc[i] > self.tol]

        if len(dropping_candidates) == 0: # ray termination: no variable can drop, so this LCP has no solution
            return None

        # ordinary minimum ratio test, keeping every (near-)tied minimum row
        min_ratio = min(Bq[i] / Bc[i] for i in dropping_candidates)
        tied = [i for i in dropping_candidates if Bq[i] / Bc[i] <= min_ratio + self.tol]

        # break ties lexicographically on the rows of B_inv (each divided by Bc): compare the first column,
        # then the next, and so on, keeping the smallest. Because B_inv is invertible its rows are all
        # distinct, so some column always yields a unique winner - which is what prevents cycling.
        col = 0
        while len(tied) > 1 and col < self.n:
            best = min(B_inv[i, col] / Bc[i] for i in tied)
            tied = [i for i in tied if B_inv[i, col] / Bc[i] <= best + self.tol]
            col += 1

        return tied[0]

    # TODO: apply optimizations from Chapter 4.8
    def lemke(self): # returns (w, z)
        self.n = self.q.shape[0]
        if self.M.shape != (self.n,self.n) or self.q.shape != (self.n,):
            raise ValueError('Lemke: matrix M has dimensions', self.M.shape, 'and vector q has dimensions', self.q.shape)

        y_ids = np.arange(self.n+1, 2*self.n+1) # y_ids saves the indices of the variables in the basic vector y
        self.full_B = np.block([-np.ones((self.n,1)), -self.M, np.identity(self.n)])
        B = np.identity(self.n)
        B_inv = np.identity(self.n)

        entering_i = 0 # entering_i is the index of the column of matrix full_B to be inserted; goes from 0 to 2*n
        dropping_i = np.argmin(self.q) # dropping_i is the index of the column of matrix B to be removed; goes from 0 to n-1
        if self.q[dropping_i] >= -self.tol: # if all elements of q are non-negative (within tolerance), the solution is trivial
            return (np.maximum(self.q, 0), np.zeros((self.n,))) # clamp away any tiny negative roundoff so w stays feasible

        # first pivot: the artificial variable z0 enters, the most infeasible row (most negative q) drops
        c = self.full_B[:, entering_i]
        B_inv = self.lemke_update_inverse(B_inv, B, dropping_i, c)
        y_ids[dropping_i] = entering_i
        B[:, dropping_i] = c
        entering_i = dropping_i + 1 # next entering variable is the complement of the w that just left

        for _ in range(int(self.max_iterations)):
            c = self.full_B[:, entering_i]
            Bc = B_inv @ c
            Bq = B_inv @ self.q

            dropping_i = self.lemke_min_ratio(Bc, Bq, B_inv)
            if dropping_i == None:
                raise RuntimeError('Lemke Solver failed to find a solution')

            if self.debug: self.lemke_debug(entering_i, dropping_i, y_ids, B)

            dropped_id = y_ids[dropping_i]
            B_inv = self.lemke_update_inverse(B_inv, B, dropping_i, c)
            y_ids, B = self.lemke_pivot(entering_i, dropping_i, y_ids, B, c)

            if dropped_id == 0: # z0 left the basis, so we have found a solution
                break

            entering_i = self.lemke_complimentary(dropped_id, self.n) # next entering variable is the complement of the one that just left
        else:
            raise RuntimeError('Lemke Solver exceeded max iterations')

        y, _, _, _ = np.linalg.lstsq(B, self.q)
        #y = np.linalg.solve(B, self.q)

        z = np.zeros((self.n,))
        w = np.zeros((self.n,))
        for i in range(self.n):
            yid = y_ids[i]
            if yid <= self.n:
                z[yid-1] = y[i]
            else:
                w[yid-self.n-1] = y[i]

        return (w, z)

    def lemke4mixed(self): # returns (w, x, z)
        n = self.q.shape[0]
        if self.M.shape != (n,n) or self.q.shape != (n,):
            raise ValueError('Lemke: matrix M has dimensions', self.M.shape, 'and vector q has dimensions', self.q.shape)
        if self.eq_mask.shape != (n,):
            raise ValueError('Lemke: equality mask has dimensions', self.eq_mask.shape, 'and should have', (n,))

        ineq_mask = ~self.eq_mask
        if not ineq_mask.any():
            return (np.array([]), np.linalg.pinv(self.M) @ -self.q, np.array([]))

        # select rows, then columns from that result, so each mask is only ever used on one axis at a time
        P = self.M[self.eq_mask][:,self.eq_mask]
        Q = self.M[self.eq_mask][:,ineq_mask]
        R = self.M[ineq_mask][:,self.eq_mask]
        S = self.M[ineq_mask][:,ineq_mask]

        u = self.q[self.eq_mask]
        v = self.q[ineq_mask]

        P_inv = np.linalg.pinv(P)
        RP = R @ P_inv

        self.M = S - RP @ Q
        self.q = v - RP @ u

        sol = self.lemke()
        if sol == None:
            return None
        w, z = sol
        x = -P_inv @ (u + Q @ z)

        return (w, x, z)


    def lemke_debug(self, entering_i, dropping_i, y_ids, B): # TODO: test this once inequalities are added and the true mixed lcp is solved
        print('----------------')
        print('Iteration', self.iteration)
        print()

        n = y_ids.shape[0]

        print('y = [ ', end='')
        for yi in y_ids:
            print(self.lemke_yid2str(yi, n), end=' ')
        print(']')


        print('B:')
        print(B)

        print()

        print('entering:', self.lemke_yid2str(entering_i, n))
        print('dropping:', self.lemke_yid2str(y_ids[dropping_i], n))
        print('dropping_i:', dropping_i)

        print()

    def lemke_yid2str(self, yid, n):
        if yid <= n:
            return 'z' + str(yid)
        elif yid <= 2*n:
            return 'w' + str(yid-n)

    def lemke_str2yid(self, var, n):
        if var[0] == 'z':
            return int(var[1:])
        elif var[0] == 'w':
            return int(var[1:]) + n
        else:
            return -1

    def lemke_complimentary(self, yid, n):
        if yid <= n:
            return yid + n
        elif yid <= 2*n:
            return yid - n

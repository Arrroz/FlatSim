import numpy as np

class Solver():

    def __init__(self, max_iterations=1e5, debug=False):
        self.M = np.zeros((0,0))
        self.q = np.zeros((0,))
        self.num_eqs = 0

        self.debug = debug
        self.iteration = 0
        self.max_iterations = max_iterations

    def solve(self):
        pass


class LemkeSolver(Solver):

    def solve(self):
        self.iteration = 0

        # TODO: cheating to avoid numerical errors; debug better to find if it's really necessary
        for i in range(self.q.shape[0]):
            if abs(self.q[i]) < 1e-3:
                self.q[i] = 0
        for i in range(self.M.shape[0]):
            for j in range(self.M.shape[1]):
                if abs(self.M[i,j]) < 1e-3:
                    self.M[i,j] = 0

        return self.lemke4mixed()
    
    def lemke_pivot(self, entering_i, dropping_i, y_ids, B, c):
        new_B = np.copy(B)
        new_B[:, dropping_i] = c

        new_y_ids = np.copy(y_ids)
        new_y_ids[dropping_i] = entering_i

        return (new_y_ids, new_B)

    
    def lemke_iteration(self, entering_i, y_ids, B):
        if tuple(y_ids) in self.visited_nodes: # TODO: inefficient as it considers a different order of y_ids as a different node, making it get tested again
            return None
        self.visited_nodes.append(tuple(y_ids))

        c = self.full_B[:, entering_i]

        B_inv = np.linalg.pinv(B)
        #B_inv = np.linalg.inv(B)
        Bc = np.matmul(B_inv, c)
        Bq = np.matmul(B_inv, self.q)

        ratios = np.array([Bq[i] / Bc[i] for i in range(self.n)])
        dropping_candidates = np.array([True for i in range(self.n)])

        for i in range(self.n): # TODO: made the below constant negative; should allow for more solutions to be searched but also some slight constraint ignoring; why is it making it so that the solver fails more often???
            if ratios[i] <= 1e-9: # in theory, only negative values should be excluded; however, values close to 0 could be positive due to numerical error: those should be excluded as well; the chosen threshold was determined empirically (looking at specific values for ratios)
                dropping_candidates[i] = False

        if dropping_candidates.sum() == 0: # dead end
            return None
        
        min_ratio = np.min(ratios, initial = ratios[dropping_candidates][0], where=dropping_candidates)
        for i in range(self.n):
            if ratios[i] - min_ratio > 1e-9: # TODO: don't forget to put this tolerance (which shows up elsewhere) as a constant
                dropping_candidates[i] = False

        for dropping_i in range(self.n): # try removing z0 first; avoids extra iterations in case of tie
            if dropping_candidates[dropping_i] and y_ids[dropping_i] == 0:
                if self.debug: self.lemke_debug(entering_i, dropping_i, y_ids, B)
                return self.lemke_pivot(entering_i, dropping_i, y_ids, B, c)

        for dropping_i in range(self.n-1, -1, -1): # going through the range the other way around to start by trying to remove the w variables
            if not dropping_candidates[dropping_i]:
                continue

            if self.debug: self.lemke_debug(entering_i, dropping_i, y_ids, B)
            
            self.iteration += 1
            if self.iteration > self.max_iterations:
                return None
            
            next_entering_i = self.lemke_complimentary(y_ids[dropping_i], self.n) # get complimentary variable to the dropping one; this is the next entering variable
            new_y_ids, new_B = self.lemke_pivot(entering_i, dropping_i, y_ids, B, c) # get the state of the next iteration

            sol = self.lemke_iteration(next_entering_i, new_y_ids, new_B)
            if sol != None:
                return sol # return the solution if found through this iteration
            
            if self.debug: print('------  backtrack  ------') # inform that it failed to find a solution with the previous pivot
            
        return None # no solutions found in this iteration

    # TODO: apply optimizations from Chapter 4.8
    def lemke(self): # returns (w, z)
        self.n = self.q.shape[0]
        if self.M.shape != (self.n,self.n) or self.q.shape != (self.n,):
            raise ValueError('Lemke: matrix M has dimensions', self.M.shape, 'and vector q has dimensions', self.q.shape)
        
        self.visited_nodes = [] # used to avoid infinite recursion

        y_ids = np.arange(self.n+1, 2*self.n+1) # y_ids saves the indices of the variables in the basic vector y
        self.full_B = np.block([-np.ones((self.n,1)), -self.M, np.identity(self.n)])
        B = np.identity(self.n)
        
        entering_i = 0 # entering_i is the index of the column of matrix full_B to be inserted; goes from 0 to 2*n
        dropping_i = np.argmin(self.q) # dropping_i is the index of the column of matrix B to be removed; goes from 0 to n-1
        if self.q[dropping_i] >= 0: # if all elements of q are non-negative, the solution is trivial
            return (self.q, np.zeros((self.n,)))

        y_ids[dropping_i] = entering_i
        B[:, dropping_i] = self.full_B[:, entering_i]

        entering_i = dropping_i + 1
        
        sol = self.lemke_iteration(entering_i, y_ids, B)
        if sol == None:
            raise RuntimeError('Lemke Solver failed to find a solution')
        
        y_ids, B = sol

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
        if self.num_eqs > n:
            raise ValueError('Lemke: number of equalities is', self.num_eqs, 'and number of total constraints is only', n)

        if self.num_eqs == n:
            return (np.array([]), np.matmul(np.linalg.inv(self.M), -self.q), np.array([]))

        P = self.M[:self.num_eqs,:self.num_eqs]
        Q = self.M[:self.num_eqs,self.num_eqs:]
        R = self.M[self.num_eqs:,:self.num_eqs]
        S = self.M[self.num_eqs:,self.num_eqs:]

        u = self.q[:self.num_eqs]
        v = self.q[self.num_eqs:]

        P_inv = np.linalg.inv(P)
        RP = np.matmul(R, P_inv)

        self.M = S - np.matmul(RP, Q)
        self.q = v - np.matmul(RP, u)

        sol = self.lemke()
        if sol == None:
            return None
        w, z = sol
        x = -np.matmul(P_inv, u + np.matmul(Q, z))

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

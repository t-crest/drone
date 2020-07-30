#!/usr/bin/env python3

# Copyright (c) 2020, Michael Platzer (TU Wien)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# SPDX-License-Identifier: BSD-2-Clause


import numpy as np
import sympy as sp
import sympy.printing.latex as spltx

################################################################################
# GENERATE REPORT FILE:

with open('raffo_params.md', 'w') as mdf:
    mdf.write('% Report on the Calculation of Raffo-Controller Parameters\n\n')

    # System parameters:
    mdf.write('# System parameters\n\n')

    m, g, L, r, alpha = sp.symbols('m g L r alpha')
    I_xx, I_yy, I_zz = sp.symbols('I_xx I_yy I_zz')
    # system_params = [ (m, 0.74), (L, 0.21),
                      # (r, 0.000006 / 0.000029),
                      # (alpha, 5 * sp.pi / 180),
                      # (g, 9.81),
                      # (I_xx, 0.004), (I_yy, 0.004), (I_zz, 0.0084) ]
    system_params = [ (m, 1.28 + 0.11 * 4), (L, 0.263),
                      (r, 0.2), # (r, 0.5) (r, 0.000006 / 0.000029),
                      (alpha, 10 * sp.pi / 180),
                      (g, 9.81),
                      #(I_xx, 0.0135), (I_yy, 0.0135), (I_zz, 0.0246) ]
                      (I_xx, 0.0135 + 0.0029226 * 4),
                      (I_yy, 0.0135 + 0.0029226 * 4),
                      (I_zz, 0.0246 + 0.005819 * 4) ]

    # added weights: 4 weights with 0.11 Kg each, one on each arm, located at
    #                a radius of 0.23 m from the center of gravity;
    #                 => distance from x- and y-axis: 0.163 m
    #                 => added moment of inertia of one weight around:
    #                  * x- or y-axis: 0.163^2 * 0.11 = 0.0029226 Kg m^2
    #                  * z-axis: 0.23^2 * 0.11 = 0.005819 Kg m^2
    #                 => the added weights approximately double the moments

    mdf.write('Universal constants:\n\n$$'
              'g = ' + spltx(g.subs(system_params).evalf(5)) + '$$\n\n'
              'Design parameters:\n\n$$'
              'm = ' + spltx(m.subs(system_params).evalf(5)) + '\\qquad '
              'L = ' + spltx(L.subs(system_params).evalf(5)) + '\\qquad '
              'r = ' + spltx(r.subs(system_params).evalf(5)) + '\\qquad '
              '\\alpha = ' + spltx(alpha.subs(system_params).evalf(5)) + ' = ' +
              spltx((alpha * 180 / sp.pi).subs(system_params).evalf(5)) +
              '^\\circ \\qquad I = \\left[\\begin{matrix}' +
              spltx(I_xx.subs(system_params).evalf(5)) + ' \\\\ ' +
              spltx(I_yy.subs(system_params).evalf(5)) + ' \\\\ ' +
              spltx(I_zz.subs(system_params).evalf(5)) +
              '\\end{matrix}\\right]$$\n\n')

    sa, ca = sp.sin(alpha), sp.cos(alpha)
    # B = sp.Matrix([[  0  , L*ca,  0  ,-L*ca ],
                   # [-L*ca,  0  , L*ca,  0   ],
                   # [ r*ca,-r*ca, r*ca,-r*ca ],
                   # [ -sa ,  0  ,  sa ,  0   ],
                   # [  0  , -sa ,  0  ,  sa  ],
                   # [  ca ,  ca ,  ca ,  ca  ]])
    #sqLc = sp.sqrt(L)*ca
    sqLc = sp.sqrt(2)*L*ca
    sq2s = sp.sqrt(2)*sa
    B = sp.Matrix([[ sqLc, sqLc,-sqLc,-sqLc ],
                   [-sqLc, sqLc,-sqLc, sqLc ],
                   [-r*ca, r*ca, r*ca,-r*ca ],
                   [-sq2s, sq2s,-sq2s, sq2s ],
                   [-sq2s,-sq2s, sq2s, sq2s ],
                   [  ca ,  ca ,  ca ,  ca  ]])

    mdf.write('## Input coupling matrix:\n\n'
              #'$$\\begin{split}\n'
              #'B & = ' + spltx(B) + '\\\\\n'
              #'  & = ' + spltx(B.subs(system_params).evalf(5)) + '\\\\\n'
              #'\\end{split}$$\n\n')
              '$$B = ' + spltx(B) + ' = ' +
              spltx(B.subs(system_params).evalf(5)) + '$$\n\n')

    spBpi = B.subs(system_params).evalf().pinv()

    B = np.array(B.subs(system_params).evalf().tolist(), dtype=np.float64)
    Bpi = np.linalg.pinv(B)

    mdf.write('Pseudo-inverse (Moore–Penrose inverse) '
              'of the input coupling matrix B:\n\n'
              '$$\\begin{split}\n'
              'B^\\# & = ' + spltx(spBpi.evalf(5)) +
              '\\rightarrow\\text{sympy}\\\\\n'
              '      & = ' + spltx(sp.Matrix(Bpi).evalf(5)) +
              '\\rightarrow\\text{numpy}\\\\\n'
              '\\end{split}$$\n\n')

    # Inertia and Coriolis matrices:

    I_B = sp.diag(I_xx, I_yy, I_zz)

    #omega = sp.Matrix(sp.symbols('p q r'))
    omega = sp.Matrix(sp.symbols('p_omega q_omega r_omega'))
    omega_X = sp.Matrix([[0, -omega[2], omega[1]],
                         [omega[2], 0, -omega[0]],
                         [-omega[1], omega[0], 0]])

    C_B = omega_X * I_B

    mdf.write('## Inertia and Coriolis matrices:\n\n'
              'Angular rate vector (body frame): '
              '$\\omega = ' + spltx(omega.T) + '^T$\n\n'
              '$$I_B = ' + spltx(I_B) + ' = ' +
              spltx(I_B.subs(system_params).evalf(5)) + ' \\qquad \\qquad '
              'C_B = ' + spltx(C_B) + ' = ' +
              spltx(C_B.subs(system_params).evalf(5)) + '$$\n\n')

    # Newton-Euler equation matrices:
    M = sp.diag(I_B, m * sp.eye(3))
    C = sp.diag(C_B, sp.zeros(3))
    G = sp.Matrix([0, 0, 0, 0, 0, m * g])

    # Separation in controlled and uncontrolled degrees of freedom:
    M_uu, M_uc, M_cu, M_cc = M[:2,:2], M[:2,2:], M[2:,:2], M[2:,2:]

    M_su, M_rc = M[:2,:2], M[2:,2:]
    C_su, C_sc, C_ru, C_rc = C[:2,:2], C[:2,2:], C[2:,:2], C[2:,2:]
    G_su, G_rc = G[:2,:], G[2:,:]

    # Controller parameters and gain matrices:
    w_1s, w_1c, w_2c, w_3c, w_us, w_uc, gamma = sp.symbols(
            'omega_1s omega_1c omega_2c omega_3c omega_us omega_uc gamma')

    rho, nu, mu, lamb = sp.symbols('rho nu mu lambda')
    ricatti_vals = [
        (rho,  gamma * w_us * w_1s / sp.sqrt(gamma**2 - w_us**2)),
        (nu,   gamma * w_uc * w_1c / sp.sqrt(gamma**2 - w_uc**2)),
        (lamb, gamma * w_uc * w_3c / sp.sqrt(gamma**2 - w_uc**2)),
        (mu,   gamma * w_uc * sp.sqrt(w_2c**2 + 2 * w_1c * w_3c) / sp.sqrt(gamma**2 - w_uc**2))
    ]

    mdf.write('# Controller parameters\n\n')

    mdf.write('Ricatti values:\n\n$$'
              '\\rho    = ' + spltx(rho.subs(ricatti_vals))  + ' \\qquad '
              '\\nu     = ' + spltx(nu.subs(ricatti_vals))   + '\\qquad '
              '\\lambda = ' + spltx(lamb.subs(ricatti_vals)) + ' \\qquad '
              '\\mu     = ' + spltx(mu.subs(ricatti_vals))   + '$$\n\n')

    R_u, R_c = (w_us**2) * sp.eye(2), (w_uc**2) * sp.eye(4)
    T_11, T_22, T_23, T_24 = rho * sp.eye(2), nu * sp.eye(4), mu * sp.eye(4), lamb * sp.eye(4)

    K_Dsu = (T_11**-1) * (M_su**-1) * (C_su * T_11 + (R_u**-1) * T_11)
    K_Dsc = (T_11**-1) * (M_su**-1) * (C_sc * T_22 - M_uc * (M_cc**-1) * (R_c**-1) * T_22)
    K_Psc = (T_11**-1) * (M_su**-1) * (C_sc * T_23 - M_uc * (M_cc**-1) * (R_c**-1) * T_23)
    K_Isc = (T_11**-1) * (M_su**-1) * (C_sc * T_24 - M_uc * (M_cc**-1) * (R_c**-1) * T_24)
    K_Dru = (T_22**-1) * (M_rc**-1) * (C_ru * T_11 - M_cu * (M_uu**-1) * (R_u**-1) * T_11)
    K_Drc = (T_22**-1) * (M_rc**-1) * (C_rc * T_22 + (R_c**-1) * T_22) + (T_22**-1) * T_23
    K_Prc = (T_22**-1) * (M_rc**-1) * (C_rc * T_23 + (R_c**-1) * T_23) + (T_22**-1) * T_24
    K_Irc = (T_22**-1) * (M_rc**-1) * (C_rc * T_24 + (R_c**-1) * T_23)

    K_D = sp.Matrix(sp.BlockMatrix([[ K_Dsu, K_Dsc ], [ K_Dru, K_Drc ]]))
    K_P = sp.Matrix(sp.BlockMatrix([[ sp.zeros(2,2), K_Psc ], [ sp.zeros(4,2), K_Prc ]]))
    K_I = sp.Matrix(sp.BlockMatrix([[ sp.zeros(2,2), K_Isc ], [ sp.zeros(4,2), K_Irc ]]))

    # Simplify the components of the gain matrices:

    K_Dsc_factor = nu / rho # sp.gcd(tuple(K_Dsc))
    K_Dsc_rest = K_Dsc / K_Dsc_factor
    K_Dru_factor = rho / nu # sp.gcd(tuple(K_Dru))
    K_Dru_rest = K_Dru / K_Dru_factor

    K_Drc_add = mu / nu # K_Drc[0].args[0]    # get first argument of sympy.core.add
    K_Drc_rest = K_Drc - K_Drc_add * sp.eye(4)
    K_Drc_factor = 1 / (w_uc**2) # sp.gcd(tuple(K_Drc_rest))
    K_Drc_rest = K_Drc_rest / K_Drc_factor

    K_Psc_factor = mu / rho # sp.gcd(tuple(K_Psc))
    K_Psc_rest = K_Psc / K_Psc_factor

    K_Prc_add = lamb / nu # K_Prc[0].args[0]    # get first argument of sympy.core.add
    K_Prc_rest = K_Prc - K_Prc_add * sp.eye(4)
    K_Prc_factor = mu / (nu * w_uc**2) # sp.gcd(tuple(K_Prc_rest))
    K_Prc_rest = K_Prc_rest / K_Prc_factor

    K_Isc_factor = lamb / rho # sp.gcd(tuple(K_Isc))
    K_Isc_rest = K_Isc / K_Isc_factor
    K_Irc_factor = mu / (nu * w_uc**2) # sp.gcd(tuple(K_Irc))
    K_Irc_rest = K_Irc / K_Irc_factor

    mdf.write('## Gain matrices:\n\n'
              '$$\\begin{array}{r c@{\\quad=\\quad}c}\n'
              #'K_D & ' + spltx(K_D) + '\\\\\n'
              'K_D = & \\left[\\begin{matrix}' + spltx(K_Dsu) + ' & ' +
              spltx(K_Dsc_factor) + spltx(K_Dsc_rest) + '\\\\' +
              spltx(K_Dru_factor) + spltx(K_Dru_rest) + ' & ' +
              spltx(K_Drc_add) + '\\textbf{1} +' +
              spltx(K_Drc_factor) + spltx(K_Drc_rest) +
              '\\end{matrix}\\right]\n'
              '      & \\left[\\begin{matrix}' +
              spltx(K_Dsu.subs(ricatti_vals)) + ' & ' +
              spltx(K_Dsc_factor.subs(ricatti_vals)) + spltx(K_Dsc_rest) + '\\\\' +
              spltx(K_Dru_factor.subs(ricatti_vals)) + spltx(K_Dru_rest) + ' & ' +
              spltx(K_Drc_add.subs(ricatti_vals)) + '\\textbf{1} +' +
              spltx(K_Drc_factor) + spltx(K_Drc_rest) +
              '\\end{matrix}\\right] \\\\ \\\\[2mm]\n'

              'K_P = & \\left[\\begin{matrix} \\quad \\textbf{0} \\quad & ' +
              spltx(K_Psc_factor) + spltx(K_Psc_rest) + '\\\\'
              '\\textbf{0} & ' +
              spltx(K_Prc_add) + '\\textbf{1} +' +
              spltx(K_Prc_factor) + spltx(K_Prc_rest) +
              '\\end{matrix}\\right]\n'
              '      & \\left[\\begin{matrix} \\quad \\textbf{0} \\quad & ' +
              spltx(K_Psc_factor.subs(ricatti_vals)) + spltx(K_Psc_rest) + '\\\\' +
              '\\textbf{0} & ' +
              spltx(K_Prc_add.subs(ricatti_vals)) + '\\textbf{1} +' +
              spltx(K_Prc_factor.subs(ricatti_vals)) + spltx(K_Prc_rest) +
              '\\end{matrix}\\right] \\\\ \\\\[2mm]\n'

              'K_I = & \\left[\\begin{matrix} \\quad \\textbf{0} \\quad & ' +
              spltx(K_Isc_factor) + spltx(K_Isc_rest) + '\\\\'
              '\\textbf{0} & ' +
              spltx(K_Irc_factor) + spltx(K_Irc_rest) +
              '\\end{matrix}\\right]\n'
              '      & \\left[\\begin{matrix} \\quad \\textbf{0} \\quad & ' +
              spltx(K_Isc_factor.subs(ricatti_vals)) + spltx(K_Isc_rest) + '\\\\' +
              '\\textbf{0} & ' +
              spltx(K_Irc_factor.subs(ricatti_vals)) + spltx(K_Irc_rest) +
              '\\end{matrix}\\right] \\\\\n'
              '\\end{array}$$\n\n')

    # Reason about proportional, integral and derivative gains:

    w_xi1, w_xi2, w_xi3 = sp.symbols('\\omega_{\\xi1} \\omega_{\\xi2} \\omega_{\\xi3}',
                                     real=True)

    mdf.write('## Position error gain:\n\n'
              'In the gain matrices we observe that $K_{D\\xi}$, $K_{P\\xi}$ '
              'and $K_{I\\xi}$, i.e. the derivative, proportional and '
              'integral gain of the position error $\\xi_{err}$ respectively, '
              'equal:\n\n'
              '$$K_{D\\xi} = ' + spltx(K_Drc_add + K_Drc_factor) + '\\cdot '
              '\\frac{1}{m} \\qquad '
              'K_{P\\xi} = ' + spltx(K_Prc_add + K_Prc_factor) + '\\cdot '
              '\\frac{1}{m} \\qquad '
              'K_{I\\xi} = ' + spltx(K_Irc_factor) + '\\cdot \\frac{1}{m}'
              '$$\n\n')

    mdf.write('Let: $$\\omega_{\\xi1} = ' + spltx(K_Drc_factor) + ' \\qquad ' +
              '\\omega_{\\xi2} = ' + spltx(K_Drc_add) + ' = ' +
              spltx(K_Drc_add.subs(ricatti_vals)) + ' \\qquad ' +
              '\\omega_{\\xi3} = ' + spltx(K_Prc_add) + ' = ' +
              spltx(K_Prc_add.subs(ricatti_vals)) + '$$\n\n')

    mdf.write('We would like to use $\\omega_{\\xi1}$, $\\omega_{\\xi2}$ and '
              '$\\omega_{\\xi3}$ to choose the derivative, proportional and '
              'integral gains of the regulated and controlled degrees of '
              'freedom. Therefore, we solve these equations '
              'for $\\omega_{uc}$, $\\omega_{1c}$ and $\\omega_{3c}$:\n\n')

    w_uc_eq, w_1c_eq, w_3c_eq = next(iter(sp.nonlinsolve([
        w_xi1 - K_Drc_factor.subs(ricatti_vals),
        w_xi2 - K_Drc_add.subs(ricatti_vals),
        w_xi3 - K_Prc_add.subs(ricatti_vals)
    ], [ w_uc, w_1c, w_3c ])))

    mdf.write('$$\\omega_{uc} = ' + spltx(w_uc_eq) + ' \\qquad ' +
              '\\omega_{1c} = ' + spltx(w_1c_eq) + ' \\qquad ' +
              '\\omega_{3c} = ' + spltx(w_3c_eq) + '$$\n\n')

    mdf.write('All values must be real, thus $\\omega_{\\xi1} > 0$ must hold. '
              'Another important constraint is that '
              '$\\omega_{\\xi2}^2 > 2 \\omega_{\\xi3}$. '
              'Note that this constraint is trivial if '
              '$\\omega_{\\xi3} < 0$.\n\n')

    mdf.write('Note that $\\omega_{\\xi1}$, $\\omega_{\\xi2}$ and '
              '$\\omega_{\\xi3}$ have been chosen such that the effective '
              'derivative, proportional and integral gains of the position '
              'error $\\xi_{err}$ are:\n\n')

    mdf.write('$$K_{D\\xi} = ' + spltx(w_xi2 + w_xi1 / m) + ' \\qquad ' +
              'K_{P\\xi} = ' + spltx(w_xi3 + w_xi1 * w_xi2 / m) + ' \\qquad ' +
              'K_{I\\xi} = ' + spltx(w_xi1 * w_xi2 / m) + '$$\n\n')

    K_Dxi, K_Pxi, K_Ixi = sp.symbols('K_{D\\xi} K_{P\\xi} K_{I\\xi}')

    w_xi_sols = list(iter(sp.nonlinsolve([
        K_Dxi - (w_xi2 + w_xi1 / m),
        K_Pxi - (w_xi3 + w_xi1 * w_xi2 / m),
        K_Ixi - (w_xi1 * w_xi2 / m)
    ], [ w_xi1, w_xi2, w_xi3 ])))

    w_xi1_eq1, w_xi2_eq1, w_xi3_eq1 = w_xi_sols[0]
    w_xi1_eq2, w_xi2_eq2, w_xi3_eq2 = w_xi_sols[1]

    mdf.write('Solving these equations for $\\omega_{\\xi1}$, $\\omega_{\\xi2}$ '
              'and $\\omega_{\\xi3}$ yields two possible solutions:\n\n'
              '$$\\left\\{ \\begin{array}{l}\n'
              '\\omega_{\\xi1} = ' + spltx(w_xi1_eq1) + ' \\qquad ' +
              '\\omega_{\\xi2} = ' + spltx(w_xi2_eq1) + ' \\qquad ' +
              '\\omega_{\\xi3} = ' + spltx(w_xi3_eq1) + ' \\\\ \\\\\n'
              '\\omega_{\\xi1} = ' + spltx(w_xi1_eq2) + ' \\qquad ' +
              '\\omega_{\\xi2} = ' + spltx(w_xi2_eq2) + ' \\qquad ' +
              '\\omega_{\\xi3} = ' + spltx(w_xi3_eq2) + '\n'
              '\\end{array} \\right.$$\n\n')

    mdf.write('We see that these results introduce an additional constraint, '
              'that $K_{D\\xi}^2 \\geq 4 K_{I\\xi}$. Note that the constraint '
              '$\\omega_{\\xi1} > 0$ is now trivial (as long as $K_{D\\xi} > 0$ '
              'and $K_{I\\xi} > 0$), whereas the constraint '
              '$\\omega_{\\xi2}^2 > 2 \\omega{rc3}$ now becomes:\n\n'
              '$$\\left\\{ \\begin{array}{l}\n' +
              spltx(w_xi2_eq1**2) + ' > 2 \\, (' + spltx(w_xi3_eq1) +
              ') \\qquad \\Rightarrow \qquad' +
              spltx((w_xi2_eq1**2).simplify() / 2) + ' > ' +
              spltx(w_xi3_eq1) + ' \\\\ \\\\\n' +
              spltx(w_xi2_eq2**2) + ' > 2 \\, (' + spltx(w_xi3_eq2) +
              ') \\qquad \\Rightarrow \qquad' +
              spltx((w_xi2_eq2**2).simplify() / 2) + ' > ' +
              spltx(w_xi3_eq2) + '\n'
              '\\end{array} \\right.$$\n\n')

    mdf.write('Note that when $K_{D\\xi}^2 = 4 K_{I\\xi}$, '
              'then both of these equations become identical and yield '
              '$\\frac{1}{8} K_{D\\xi}^2 > ' + spltx(w_xi3_eq1) +
              ' \\Rightarrow \\frac{1}{2} K_{I\\xi} > ' + spltx(w_xi3_eq1) +
              ' \\Rightarrow K_{P\\xi} < \\frac{3}{2} K_{I\\xi}$. '
              'As $K_{D\\xi}^2$ grows larger than $4 K_{I\\xi}$, the first '
              'equation forces a lower bound on $K_{P\\xi}$ than the second '
              'one; hence the second equations gives more freedom in choosing '
              '$K_{P\\xi}$. For $K_{D\\xi} \\gg K_{I\\xi}$ the two equations '
              'can be approximated by:\n\n'
              '$$\\left\\{ \\begin{array}{l}\n'
              '0 > ' + spltx(w_xi3_eq1) + ' \\Rightarrow '
              'K_{P\\xi} < K_{I\\xi} \\\\ \\\\\n'
              '\\frac{(2 K_{D\\xi})^2}{8} > ' + spltx(w_xi3_eq1) +
              ' \\Rightarrow '
              'K_{P\\xi} < K_{I\\xi} + \\frac{1}{2} K_{D\\xi}^2\n'
              '\\end{array} \\right.$$\n\n')

    mdf.write('Hence we see that for $K_{D\\xi} \\gg K_{I\\xi}$, the upper '
              'bound for $K_{P\\xi}$ approaches $\\frac{1}{2} K_{D\\xi}^2$, '
              'which we can also observed in the following plot:\n\n')

    import matplotlib.pyplot as plt

    for Ki in [ 0., .25, .5, .75, 1., 1.5, 2., 3., 4. ]:
        w_xi2_val = w_xi2_eq2.subs([ (K_Ixi, Ki) ] + system_params)
        Kd_vals = list(np.arange(np.sqrt(4 * Ki) + .01, 4.5, .1))
        Kp_constr1 = [ w_xi2_val.subs([ (K_Dxi, Kd) ])**2 / 2 + Ki for Kd in Kd_vals ]
        plt.plot([ np.sqrt(4 * Ki) ] + Kd_vals, [ 0 ] + Kp_constr1, label=('Ki = ' + str(Ki)))
    plt.xlabel('Kd')
    plt.ylabel('Kp')
    plt.legend()
    #plt.show()
    #plt.gca().set_position([0, 0, 1, 1])
    #plt.savefig('raffo_params_possible_gains.pdf')

    mdf.write('![Upper bound of $K_{P\\xi}$ in function of $K_{D\\xi}$ '
              'for different values of $K_{I\\xi}$. Note that $K_{D\\xi}$ can '
              'always be increased without limit, but has a minimum value '
              'depending on $K_{I\\xi}$ (i.e. $K_{D\\xi}^2 > 4 K_{I\\xi}$). '
              '$K_{P\\xi}$ however can always be decreased to 0, but its '
              'maximum value is limited.]'
              '(raffo_params_possible_gains.pdf){ width=50% }\n\n')

    # valid_gains = []
    # for Kd, Ki in np.mgrid[0:5:.1,0:5:.1].reshape(2,-1).T:
        # test_gains = [ (K_Dxi, Kd), (K_Pxi, 1.), (K_Ixi, Ki) ]
        # w_xi2_val = w_xi2_eq.subs(test_gains + system_params)
        # w_xi3_val = w_xi3_eq.subs(test_gains + system_params)
        # if Kd**2 > 4 * Ki and w_xi2_val**2 > 2 * w_xi3_val:
            # valid_gains.append((Kd, Ki))

    # from scipy.spatial import ConvexHull
    # valid_hull = [ valid_gains[idx] for idx in ConvexHull(valid_gains).vertices ]
    # print(valid_hull)

    # import matplotlib.pyplot as plt
    # plt.plot([ Kd for Kd, Ki in valid_gains ], [ Ki for Kd, Ki in valid_gains ], 'o')
    # plt.show()


    # Obtain numerical values:

    tuning_vals = [
        (w_1s, 1.5),
        #(w_1c, 1.0),
        (w_2c, 0.5),
        #(w_3c, 2.0), # (w_3c, 6.0),
        (w_us, 0.7), # (w_us, 0.7), # (w_us, 2.5),
        #(w_uc, 2.5), # (w_uc, 0.7),
        (gamma, 8.0) # 7.0) # 3.0)
    ]

    w_xi1_eq, w_xi2_eq, w_xi3_eq = w_xi1_eq2, w_xi2_eq2, w_xi3_eq2

    #xi_gain_vals = [ (K_Dxi, 2.1866), (K_Pxi, 2.2577), (K_Ixi, 0.25769) ]
    #xi_gain_vals = [ (K_Dxi, 2.2), (K_Pxi, 2.3), (K_Ixi, 0.2) ]
    #xi_gain_vals = [ (K_Dxi, 5), (K_Pxi, 2.3), (K_Ixi, 0.2) ]
    #xi_gain_vals = [ (K_Dxi, 3.4), (K_Pxi, 1.1), (K_Ixi, 0.2) ]
    #xi_gain_vals = [ (K_Dxi, 5.5), (K_Pxi, 2.4), (K_Ixi, 0.2) ]
    #xi_gain_vals = [ (K_Dxi, 6.5), (K_Pxi, 4), (K_Ixi, 0.2) ]
    #xi_gain_vals = [ (K_Dxi, 7.5), (K_Pxi, 4.6), (K_Ixi, 0.2) ]
    #xi_gain_vals = [ (K_Dxi, 9), (K_Pxi, 5.7), (K_Ixi, 0.2) ]
    #xi_gain_vals = [ (K_Dxi, 11), (K_Pxi, 7), (K_Ixi, 0.2) ]
    xi_gain_vals = [ (K_Dxi, 14), (K_Pxi, 9), (K_Ixi, 0.2) ]
    #xi_gain_vals = [ (K_Dxi, 0.), (K_Pxi, 1.), (K_Ixi, 0.) ]
    xi_gains = [ (w_xi1, w_xi1_eq), (w_xi2, w_xi2_eq), (w_xi3, w_xi3_eq) ]

    mdf.write('## Parameter values:\n\n'
              'First, we choose suitable values for the derivative, '
              'proportional and integral gains $K_{D\\xi}$, $K_{P\\xi}$ and '
              '$K_{I\\xi}$ of the position error $\\xi_{err}$. '
              'These gain values directly amplify the position error (in m) '
              'to give the control acceleration (in m$\\cdot\\text{s}^{-2}$), '
              'which adds to the gravitational acceleration vector to control '
              'the pitch and roll of the quadcopter. '
              'We choose $K_{D\\xi} = ' + spltx(K_Dxi.subs(xi_gain_vals)) +
              '$, $K_{P\\xi} = ' + spltx(K_Pxi.subs(xi_gain_vals)) +
              '$ and $K_{I\\xi} = ' + spltx(K_Ixi.subs(xi_gain_vals)) +
              '$, thus:\n\n'
              '$$\\omega_{\\xi1} = ' + spltx(w_xi1_eq) + ' = ' +
              spltx(w_xi1_eq.subs(xi_gain_vals + system_params).evalf(5)) +
              ' \\qquad \\omega_{\\xi2} = ' + spltx(w_xi2_eq) + ' = ' +
              spltx(w_xi2_eq.subs(xi_gain_vals + system_params).evalf(5)) +
              ' \\qquad \\omega_{\\xi3} = ' + spltx(w_xi3_eq) + ' = ' +
              spltx(w_xi3_eq.subs(xi_gain_vals + system_params).evalf(5)) +
              '$$\n\n')

    mdf.write('Hence we get:\n\n'
              '$$\\begin{array}{r c@{\\quad=\\quad}c@{\\quad=\\quad}l}\n'
              '\\omega_{uc} \\; = & ' + spltx(w_uc_eq) + ' & ' +
              spltx(w_uc_eq.subs(xi_gains)) + ' & ' +
              spltx(w_uc_eq.subs(xi_gains).subs(xi_gain_vals + system_params).evalf(5)) +
              '\\\\\n\\\\[1mm]\n\\omega_{1c} \\; = & ' + spltx(w_1c_eq) + ' & ' +
              spltx(w_1c_eq.subs(xi_gains)) + ' & ' +
              spltx(w_1c_eq.subs(xi_gains).subs(xi_gain_vals + system_params).evalf(5)) +
              '\\\\\n\\\\[1mm]\n\\omega_{3c} \\; = & ' + spltx(w_3c_eq) + ' & ' +
              spltx(w_3c_eq.subs(xi_gains)) + ' & ' +
              spltx(w_3c_eq.subs(xi_gains).subs(xi_gain_vals + system_params).evalf(5)) +
              '\\\\\n\\end{array}$$\n\n')

    tuning_vals = [
        (w_1s, w_1s.subs(tuning_vals)),
        (w_1c, w_1c_eq.subs(xi_gains).subs(xi_gain_vals + system_params + tuning_vals)),
        (w_2c, w_2c.subs(tuning_vals)),
        (w_3c, w_3c_eq.subs(xi_gains).subs(xi_gain_vals + system_params + tuning_vals)),
        (w_us, w_us.subs(tuning_vals)),
        (w_uc, w_uc_eq.subs(xi_gains).subs(xi_gain_vals + system_params + tuning_vals)),
        (gamma, gamma.subs(tuning_vals))
    ]

    mdf.write('## Resulting tuning values:\n\n'
              '$$\\omega_{1s} = ' + spltx(w_1s.subs(tuning_vals).evalf(5)) + '\\qquad'
              '\\omega_{1c} = ' + spltx(w_1c.subs(tuning_vals).evalf(5)) + '\\qquad'
              '\\omega_{2c} = ' + spltx(w_2c.subs(tuning_vals).evalf(5)) + '\\qquad'
              '\\omega_{3c} = ' + spltx(w_3c.subs(tuning_vals).evalf(5)) + '\\qquad'
              '\\omega_{us} = ' + spltx(w_us.subs(tuning_vals).evalf(5)) + '\\qquad'
              '\\omega_{uc} = ' + spltx(w_uc.subs(tuning_vals).evalf(5)) + '\\qquad'
              '\\gamma = ' + spltx(gamma.subs(tuning_vals)) + '$$\n\n')

    mdf.write('Ricatti values:\n\n$$'
              '\\rho    = ' + spltx(rho.subs(ricatti_vals))  + ' = ' +
              spltx(rho.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + ' \\qquad '
              '\\nu     = ' + spltx(nu.subs(ricatti_vals))   + ' = ' +
              spltx(nu.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + ' \\qquad '
              '\\lambda = ' + spltx(lamb.subs(ricatti_vals)) + ' = ' +
              spltx(lamb.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + ' \\qquad '
              '\\mu     = ' + spltx(mu.subs(ricatti_vals))   + ' = ' +
              spltx(mu.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + '$$\n\n')

    mdf.write('therefore: $$' + '\\qquad'.join(
              [ spltx(val) + ' = ' + spltx(val.subs(ricatti_vals)) + ' = ' +
                spltx(val.subs(ricatti_vals).subs(tuning_vals).evalf(5))
                for val in [
                    K_Dsc_factor, K_Dru_factor, K_Psc_factor, K_Isc_factor
                ] ]) + '$$\n\n$$' + '\\qquad'.join(
              [ spltx(val) + ' = ' + spltx(val.subs(ricatti_vals)) + ' = ' +
                spltx(val.subs(ricatti_vals).subs(tuning_vals).evalf(5))
                for val in [
                    K_Drc_add, K_Drc_factor, K_Prc_add, K_Prc_factor
                ] ]) + '$$\n\n')

    cancel_omega = [ (omega[0], 1), (omega[1], 1), (omega[2], 1) ]
    K_D_n = K_D.subs(ricatti_vals).subs(tuning_vals + cancel_omega + system_params)
    K_P_n = K_P.subs(ricatti_vals).subs(tuning_vals + cancel_omega + system_params)
    K_I_n = K_I.subs(ricatti_vals).subs(tuning_vals + cancel_omega + system_params)

    mdf.write('## Resulting gain matrices:\n\n'
              '$$\\begin{array}{r c@{\\quad=\\quad}c}\n'
              'K_D = & \\left[\\begin{matrix}' +
              spltx(K_Dsu.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + ' & ' +
              spltx(K_Dsc_factor.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + spltx(K_Dsc_rest) + '\\\\' +
              spltx(K_Dru_factor.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + spltx(K_Dru_rest) + ' & ' +
              spltx(K_Drc_add.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + '\\cdot\\textbf{1} +' +
              spltx(K_Drc_factor.subs(tuning_vals).evalf(5)) + spltx(K_Drc_rest) +
              '\\end{matrix}\\right] & ' +
              spltx(K_D.subs(ricatti_vals).subs(tuning_vals + system_params).evalf(5)) + ' \\\\ \\\\[2mm]\n'
              'K_P = & \\left[\\begin{matrix} \\quad \\textbf{0} \\quad & ' +
              spltx(K_Psc_factor.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + spltx(K_Psc_rest) + '\\\\' +
              '\\textbf{0} & ' +
              spltx(K_Prc_add.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + '\\cdot \\textbf{1} +' +
              spltx(K_Prc_factor.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + spltx(K_Prc_rest) +
              '\\end{matrix}\\right] & ' +
              spltx(K_P.subs(ricatti_vals).subs(tuning_vals + system_params).evalf(5)) + ' \\\\ \\\\[2mm]\n'
              'K_I = & \\left[\\begin{matrix} \\quad \\textbf{0} \\quad & ' +
              spltx(K_Isc_factor.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + spltx(K_Isc_rest) + '\\\\' +
              '\\textbf{0} & ' +
              spltx(K_Irc_factor.subs(ricatti_vals).subs(tuning_vals).evalf(5)) + spltx(K_Irc_rest) +
              '\\end{matrix}\\right] & ' +
              spltx(K_I.subs(ricatti_vals).subs(tuning_vals + system_params).evalf(5)) + '\\\\\n'
              '\\end{array}$$\n\n')


# write gain matrices to python file for simulation purposes:
#with open('raffo_params.py', 'w') as pyf:
#    pyf.write('# Raffo Parameters\n\n')
#    pyf.write('import numpy as np\n\n')
#    pyf.write('K_D = np.array' + str(K_D_n.evalf(8))[6:] + '\n\n')
#    pyf.write('K_P = np.array' + str(K_P_n.evalf(8))[6:] + '\n\n')
#    pyf.write('K_I = np.array' + str(K_I_n.evalf(8))[6:] + '\n\n')


################################################################################
# GENERATE PARAMETERS FILE:

import math

def fix32(vals, scale_name, scale=None):
    if scale is None:
        _, maxval = max(vals, key=lambda x: abs(x[1]))
        _, exp = math.frexp(maxval)
        scale = 31 - exp

    # macro definitions:
    lines = [ '#define ' + name for name, _ in vals + [ (scale_name, 0) ] ]
    maxlen = ((max([ len(l) for l in lines ]) + 4) // 4) * 4

    # hex values:
    lines = [ line.ljust(maxlen-1) + ((" 0x%08X" % (int(val[1] * (1<<scale))))
                 if val[1] >= 0 else ("-0x%08X" % (int(-val[1] * (1<<scale)))))
              for line, val in zip(lines, vals) ] + [ lines[-1].ljust(maxlen) ]
    maxlen = ((max([ len(l) for l in lines ]) + 4) // 4) * 4 + 4

    # add float values as comment:
    lines = [ line.ljust(maxlen) + ("// %f" % val[1])
              for line, val in zip(lines, vals) ] + [ lines[-1] + str(scale) ]
    return lines

with open('raffo_params.h', 'w') as outf:
    outf.write('#ifndef RAFFO_PARAMS_H\n')
    outf.write('#define RAFFO_PARAMS_H\n\n')
    outf.write('#include "raffo.h"\n\n')

    outf.write('/' * 80 + '\n// SYSTEM PARAMETERS:\n\n')

    outf.write('// Maximum torques (rotational component of Gamma) '
               'and torques\n' '// (translational component of Gamma):\n')
    outf.write('#define GAMMA_TORQUE_SCALE 25\n')
    outf.write('#define GAMMA_FORCE_SCALE  25\n')

    mass_str = fix32([ ('MASS', m.subs(system_params)) ], 'MASS_SCALE')
    outf.write('// Mass of the quadcopter in kg:\n' + '\n'.join(mass_str))
    outf.write('\n\n')

    gm_str = fix32([ ('G_MASS', (g*m).subs(system_params)) ],
                      'G_MASS_SCALE', 25)
    outf.write('// Gravitational force acting on the quadcopter g*mass, '
               'with g = 9.81 m/s^2;\n' '// note that the scaling factor must '
               'be the same as for GAMMA_FORCE:\n' + '\n'.join(gm_str) + '\n\n')

    inertia_str = fix32([ ('I_XX', I_xx.subs(system_params)),
                          ('I_YY', I_yy.subs(system_params)),
                          ('I_ZZ', I_zz.subs(system_params))], 'I_SCALE')
    outf.write('// Principle moments of inertia of the quadcopter:\n' +
               '\n'.join(inertia_str) + '\n\n')

    outf.write('// Pseudo-inverse (Moore–Penrose inverse) of the input '
               'coupling matrix B:\nstatic const int32_t Bpi[4][6] = {')

    maxval = max(Bpi.flatten(), key=abs)
    _, exp = math.frexp(maxval)
    scale = 31 - exp
    bpi_str = [ [ (((" 0x%08X" % (int(val * (1<<scale)))) if val >= 0 else
                    ("-0x%08X" % (int(-val * (1<<scale))))), '%f' % val)
                  for val in row ] for row in Bpi ]
    bpi_str = [ '\n//   ' + ','.join([ val[1].rjust(11) for val in row ]) +
                '\n    {' + ','.join([ val[0] for val in row ]) + ' }'
                for row in bpi_str ]
    outf.write(','.join(bpi_str))
    outf.write('\n};\n#define BPI_SCALE   ' + str(scale) + '\n\n\n')

    outf.write('/' * 80 + '\n// CONTROLLER PARAMETERS AND GAIN MATRICES:\n\n')

    tuning_str = [ name + (" = %f" % val.subs(tuning_vals)) for name, val in [
                   ('w_1s', w_1s), ('w_1c', w_1c), ('\n// w_2c', w_2c),
                   ('w_3c', w_3c), ('\n// w_us', w_us), ('w_uc', w_uc),
                   ('\n// gamma', gamma) ] ]
    outf.write('// Tuning values:\n// ' + ', '.join(tuning_str) + '\n\n')

    ricatti_str = [ equ + (" = %f" % val.subs(ricatti_vals).subs(tuning_vals))
                    for equ, val in [
        ('rho    = gamma * w_us * w_1s / sqrt(gamma^2 - w_us^2)', rho),
        ('nu     = gamma * w_uc * w_1c / sqrt(gamma^2 - w_uc^2)', nu),
        ('lambda = gamma * w_uc * w_3c / sqrt(gamma^2 - w_uc^2)', lamb),
        ('mu = gamma * w_uc * sqrt(w_2c^2 + 2 * w_1c * w_3c) / '
         'sqrt(gamma^2 - w_uc^2)\n//   ', mu)
    ] ]
    outf.write('// Ricatti values:\n// ' + '\n// '.join(ricatti_str) + '\n\n')

    outf.write('// KI matrix has following elements (constant parts only):\n')
    outf.write('//  - KI_P = I_zz * lambda / (I_xx * rho)       element [0,2]\n')
    outf.write('//  - KI_Q = -I_zz * lambda / (I_yy * rho)      element [1,2]\n')
    outf.write('//  - KI_R = mu / (I_zz * nu * w_uc^2)          element [2,2]\n')
    outf.write('//  - KI_XI = mu / (m * nu * w_uc^2)            elements [3,3] [4,4] [5,5]\n')

    KI_str  = fix32([ ('KI_P', K_I_n[0,2]),
                      ('KI_Q', K_I_n[1,2]),
                      ('KI_R', K_I_n[2,2]),
                    ], 'KI_ROT_SCALE')
    KI_str += fix32([ ('KI_XI', K_I_n[3,3]),
                    ], 'KI_XI_SCALE ')
    outf.write('\n'.join(KI_str) + '\n\n')

    outf.write('// KP matrix has following elements (constant parts only):\n')
    outf.write('//  - KP_P = I_zz * mu / (I_xx * rho)                   element [0,2]\n')
    outf.write('//  - KP_Q = -I_zz * mu / (I_yy * rho)                  element [1,2]\n')
    outf.write('//  - KP_R = lambda / nu + mu / (I_zz * nu * w_uc^2)    element [2,2]\n')
    outf.write('//  - KP_XI = lambda / nu + mu / (m * nu * w_uc^2)      elem [3,3] [4,4] [5,5]\n')

    KP_str  = fix32([ ('KP_P', K_P_n[0,2]),
                      ('KP_Q', K_P_n[1,2]),
                      ('KP_R', K_P_n[2,2]),
                    ], 'KP_ROT_SCALE')
    KP_str += fix32([ ('KP_XI', K_P_n[3,3]),
                    ], 'KP_XI_SCALE ')
    outf.write('\n'.join(KP_str) + '\n\n')

    outf.write('// KD matrix has following elements (constant parts only):\n')
    outf.write('//  - KD_PP = 1 / (I_xx * w_us^2)                       element [0,0]\n')
    outf.write('//  - KD_QQ = 1 / (I_yy * w_us^2)                       element [1,1]\n')
    outf.write('//  - KD_RR = 1 / (I_zz * w_us^2)                       element [2,2]\n')
    outf.write('//  - KD_PQR = -I_yy / I_xx + I_zz * nu / (I_xx * rho)  element [0,1] + [0,2]\n')
    outf.write('//  - KD_QPR =  I_xx / I_yy - I_zz * nu / (I_yy * rho)  element [1,0] + [1,2]\n')
    outf.write('//  - KD_RPQ = (-I_xx + I_yy) * rho / (I_zz * nu)       element [2,0] + [2,1]\n')
    outf.write('//  - KD_XI = mu / nu + 1 / (m * w_uc^2)                elem [3,3] [4,4] [5,5]\n')

    KD_str  = fix32([ ('KD_PP', K_D_n[0,0]),
                      ('KD_QQ', K_D_n[1,1]),
                      ('KD_RR', K_D_n[2,2])
                    ], 'KD_ROT1_SCALE')
    KD_str += fix32([ ('KD_PQR', K_D_n[0,1] + K_D_n[0,2]),
                      ('KD_QPR', K_D_n[1,0] + K_D_n[1,2]),
                      ('KD_RPQ', K_D_n[2,0] + K_D_n[2,1])
                    ], 'KD_ROT2_SCALE')
    KD_str += fix32([ ('KD_XI', K_D_n[3,3])
                    ], 'KD_XI_SCALE  ')
    outf.write('\n'.join(KD_str) + '\n\n')

    #outf.write('#define MAX(X, Y)   (((X) > (Y)) ? (X) : (Y))\n')
    #outf.write('#define K_MAX_SCALE (MAX(KI_SCALE, MAX(KP_SCALE, KD_SCALE)))')
    outf.write('#endif //RAFFO_PARAMS_H\n')

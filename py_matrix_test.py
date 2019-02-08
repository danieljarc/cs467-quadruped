import numpy as np

a1=1 #length from coxa to femur in cm (1 for testing)
a2=1 #length from femur to tibia in cm (1 for testing)
a3=1 #length from tibia to foot in cm (1 for testing)

T1=0 #T1 in degrees (coxa angle)
T2=0 #T2 in degrees (femur angle)
T3=0 #T3 in degrees (tibia angle)

T1=(T1/180.0)*np.pi #T1 in radians
T2=(T2/180.0)*np.pi #T2 in radians
T3=(T3/180.0)*np.pi #T3 in radians

#Build rotation matrices
R0_1=[[np.cos(T1),0,np.sin(T1)],[np.sin(T1),0,-np.cos(T1)],[0,1,0]]
R1_2=[[np.cos(T2),-np.sin(T2),0],[np.sin(T2),np.cos(T2),0],[0,0,1]]
R2_3=[[np.cos(T3),-np.sin(T3),0],[np.sin(T3),np.cos(T3),0],[0,0,1]]
R0_2=np.dot(R0_1, R1_2)
R0_3=np.dot(R0_2, R2_3)

#Build displacement vectors
d0_1=[[a1*np.cos(T1)],[a1*np.sin(T1)],[0]]
d1_2=[[a2*np.cos(T2)],[a2*np.sin(T2)],[0]]
d2_3=[[a3*np.cos(T3)],[a3*np.sin(T3)],[0]]

#Build homogeneous transformation matrices
H0_1=np.concatenate((R0_1,d0_1),1) #concatenates d0_1 on right
H0_1=np.concatenate((H0_1,[[0,0,0,1]]),0) #concatenates [0,0,0,1] on bottom
H1_2=np.concatenate((R1_2,d1_2),1)
H1_2=np.concatenate((H1_2,[[0,0,0,1]]),0)
H2_3=np.concatenate((R2_3,d2_3),1)
H2_3=np.concatenate((H2_3,[[0,0,0,1]]),0)
H0_2=np.dot(H0_1,H1_2)
H0_3=np.dot(H0_2,H2_3)

#Denavit-Hartenberg Parameter Table
PT=[[T1,(90/180.0)*np.pi,a1,0],
    [T2,0,a2,0],
    [T3,0,a3,0]]

#Check to see if DH generated HTMs match with manually calculated HTMs
i = 0
PT_H0_1=[[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
         [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
         [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
         [0,0,0,1]]
i = 1
PT_H1_2=[[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
         [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
         [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
         [0,0,0,1]]
i = 2
PT_H2_3=[[np.cos(PT[i][0]),-np.sin(PT[i][0])*np.cos(PT[i][1]),np.sin(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.cos(PT[i][0])],
         [np.sin(PT[i][0]),np.cos(PT[i][0])*np.cos(PT[i][1]),-np.cos(PT[i][0])*np.sin(PT[i][1]),PT[i][2]*np.sin(PT[i][0])],
         [0,np.sin(PT[i][1]),np.cos(PT[i][1]),PT[i][3]],
         [0,0,0,1]]

PT_H0_2=np.dot(PT_H0_1,PT_H1_2)
PT_H0_3=np.dot(PT_H0_2,PT_H2_3)

#Extracted rotation matrix R0_2 and displacement vectors d0_2, d0_3 needed for Jacobian
R0_2=[[PT_H0_2[0][0],PT_H0_2[0][1],PT_H0_2[0][2]],
      [PT_H0_2[1][0],PT_H0_2[1][1],PT_H0_2[1][2]],
      [PT_H0_2[2][0],PT_H0_2[2][1],PT_H0_2[2][2]]]

d0_2=[[PT_H0_2[0][3]],
      [PT_H0_2[1][3]],
      [PT_H0_2[2][3]]]

d0_3=[[PT_H0_3[0][3]],
      [PT_H0_3[1][3]],
      [PT_H0_3[2][3]]]

#Identity matrix used in Jacobian calculations
I=[[1,0,0],
   [0,1,0],
   [0,0,1]]

#Common multiplier used in Jacobian calculations
m=[[0],
   [0],
   [1]]

#Jacobian intermediate values
d0_3_minus_d0_0=np.subtract(d0_3,[[0],[0],[0]])
d0_3_minus_d0_1=np.subtract(d0_3,d0_1)
d0_3_minus_d0_2=np.subtract(d0_3,d0_2)
R0_0_times_m=np.dot(I,m)
R0_1_times_m=np.dot(R0_1,m)
R0_2_times_m=np.dot(R0_2,m)

#Jacobian values: J11 indicates first row, first column result, J12 indicates first row, second column result...etc
J11=np.cross(R0_0_times_m, d0_3_minus_d0_0,axis=0)
J12=np.cross(R0_1_times_m, d0_3_minus_d0_1,axis=0)
J13=np.cross(R0_2_times_m, d0_3_minus_d0_2,axis=0)
J21=R0_0_times_m
J22=R0_1_times_m
J23=R0_2_times_m

#Jacobian matrice results - need to check J11, J12, J13 for accuracy...
print("J11:")
print(np.matrix(J11))
print("\n")
print("J12:")
print(np.matrix(J12))
print("\n")
print("J13:")
print(np.matrix(J13))
print("\n")
print("J21:")
print(np.matrix(J21))
print("\n")
print("J22:")
print(np.matrix(J22))
print("\n")
print("J23:")
print(np.matrix(J23))
print("\n")

#HTM vs DH HTM checks
##print("H0_1:")
##print(np.matrix(H0_1))
##print("\n")
##print("PT_H0_1:")
##print(np.matrix(PT_H0_1))
##print("\n")
##print("H1_2:")
##print(np.matrix(H1_2))
##print("\n")
##print("PT_H1_2:")
##print(np.matrix(PT_H1_2))
##print("\n")
##print("H2_3:")
##print(np.matrix(H2_3))
##print("\n")
##print("PT_H2_3:")
##print(np.matrix(PT_H2_3))
##print("\n")
##print("H0_2:")
##print(np.matrix(H0_2))
##print("\n")
##print("PT_H0_2:")
##print(np.matrix(PT_H0_2))
##print("\n")
##print("H0_3:")
##print(np.matrix(H0_3))
##print("\n")
##print("PT_H0_3:")
##print(np.matrix(PT_H0_3))
##print("\n")

#Jacobian intermediate checks
##print("d0_3_minus_d0_0:")
##print(np.matrix(d0_3_minus_d0_0))
##print("\n")
##print("d0_3_minus_d0_1:")
##print(np.matrix(d0_3_minus_d0_1))
##print("\n")
##print("d0_3_minus_d0_2:")
##print(np.matrix(d0_3_minus_d0_2))
##print("\n")
##print("R0_0_times_m:")
##print(np.matrix(R0_0_times_m))
##print("\n")
##print("dR0_1_times_m:")
##print(np.matrix(R0_1_times_m))
##print("\n")
##print("R0_2_times_m:")
##print(np.matrix(R0_2_times_m))
##print("\n")


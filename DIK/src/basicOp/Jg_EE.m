function [Jvf,Jwf]=Jg_EE(nb,EE,T,LJoint,parent,axis)
EtaRow=cell(nb,1);
Eta=cell(nb,1);
O_f=cell(nb,1);
Jv = cell(nb,1);
Jw = cell(nb,1);
Jvf=zeros(3,nb);
Jwf=zeros(3,nb);
i=LJoint;
O_f{1}=EE-T{1}(1:3,4);
switch axis(1)
    case 1
        Jv{1}=cross([1;0;0],O_f{1});
        Jw{1}=[1;0;0];
    case 2
        Jv{1}=cross([0;1;0],O_f{1});
        Jw{1}=[0;1;0];
    case 3
        Jv{1}=cross([0;0;1],O_f{1});
        Jw{1}=[0;1;0];
    case 4
        Jv{1}=[1;0;0];
        Jw{1}=[0;0;0];
    case 5
        Jv{1}=[0;1;0];
        Jw{1}=[0;0;0];
    case 6
        Jv{1}=[0;0;1];
        Jw{1}=[0;0;0];
end
Jvf(:,1)=Jv{1};
Jwf(:,1)=Jw{1};

while i>1
        switch axis(i)       
            case 1
            Eta{i}=[1;0;0];
            case 2
            Eta{i}=[0;1;0];
            case 3
            Eta{i}=[0;0;1];
            case 4
            Eta{i}=[1;0;0];
            case 5
            Eta{i}=[0;1;0];
            case 6
            Eta{i}=[0;0;1];
        end
        EtaRow{i}=T{parent(i)}(1:3,1:3)*Eta{i};
        O_f{i}=EE-T{i}(1:3,4); %Aquisition of On-O's
if axis(i)<=3
    Jv{i}=cross(EtaRow{i},O_f{i});%Aquisition of Jv
    Jw{i}=EtaRow{i};
else
    Jv{i}=EtaRow{i};
    Jw{i}=[0;0;0];
end
Jvf(:,i)=Jv{i};
Jwf(:,i)=Jw{i};
i=parent(i);
end
end
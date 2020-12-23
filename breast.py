import torch
import scipy.io as sio
import numpy as np
import torch.nn as nn
import torch.optim as optim
from torch.autograd import Variable
from torch.nn import functional as F
import torch.utils.data as data
class Breast_data(data.Dataset):
    def __init__(self,data_set,data_label,length,mode):
        self.length=length
        self.data_set = data_set
        # self.data_set=data_set.transpose(1,0)
        self.data_label=data_label
        self.mode=mode
        self.data_set_att=np.shape(self.data_set)[1]
        for i in range(self.data_set_att):
            att_max=np.max(data_set[:,i])
            att_min=np.min(data_set[:,i])
            data_set[:,i]=(data_set[:,i]-att_min)*(1.0/(att_max-att_min))
    def __len__(self):
        return self.length
    def __getitem__(self, item):
        if self.mode == 'Test':
            return self.data_set[item].astype(np.float32)
        return self.data_set[item].astype(np.float32),\
                self.data_label[item].astype(np.float32)
class Breast_net(nn.Module):
    def __init__(self,in_channel,mlp_list):
        super(Breast_net,self).__init__()
        self.mlp=nn.ModuleList()
        last_channel=in_channel
        for out_channel in (mlp_list):
            self.mlp.append(nn.Linear(last_channel,out_channel))
            last_channel=out_channel
        self.Linear=nn.Linear(mlp_list[-1],1)
    def forward(self,breast_dataset):
        pre_label=breast_dataset
        for i in range(len(self.mlp)):
            pre_label=F.relu(self.mlp[i](pre_label))
        pre_label=self.Linear(pre_label)
        # pre_label=F.sigmoid(pre_label)
        # for i in range(10):
        #     if pre_label[i]<0.5:
        #         pre_label[i]=2
        #     else:
        #         pre_label[i]=4

        return pre_label
# class Breast_net(torch.nn.Module):
#     def __init__(self, n_feature, n_hidden, n_output, dropout=0.5):
#         super(Breast_net, self).__init__()
#         self.dropout = torch.nn.Dropout(dropout)
#
#         self.hidden_1 = torch.nn.Linear(n_feature, n_hidden)  # hidden layer
#         self.bn1 = torch.nn.BatchNorm1d(n_hidden)
#
#         self.hidden_2 = torch.nn.Linear(n_hidden, n_hidden//2)
#         self.bn2 = torch.nn.BatchNorm1d(n_hidden//2)
#
#         self.hidden_3 = torch.nn.Linear(n_hidden//2, n_hidden//4)  # hidden layer
#         self.bn3 = torch.nn.BatchNorm1d(n_hidden//4)
#
#         self.hidden_4 = torch.nn.Linear(n_hidden // 4, n_hidden // 8)  # hidden layer
#         self.bn4 = torch.nn.BatchNorm1d(n_hidden // 8)
#
#         self.out = torch.nn.Linear(n_hidden//8, n_output)  # output layer
#
#         self.hidden=torch.nn.Linear(10,30)
#         self.outt=torch.nn.Linear(30,1)
#
#     def forward(self, x):
#         # print(x.shape)
#         # x=x.view(10,1,10)
#
#         x = F.relu(self.hidden_1(x))  # activation function for hidden layer
#         x = self.dropout(self.bn1(x))
#         x = F.relu(self.hidden_2(x))  # activation function for hidden layer
#         x = self.dropout(self.bn2(x))
#         x = F.relu(self.hidden_3(x))  # activation function for hidden layer
#         x = self.dropout(self.bn3(x))
#         x = F.relu(self.hidden_4(x))  # activation function for hidden layer
#         x = self.dropout(self.bn4(x))
#         # x = F.sigmoid(self.out(x))
#         x = self.out(x)
#         # x=F.relu(self.hidden(x))
#         # x=F.sigmoid(self.outt(x))
#         return x
class Loss(nn.Module):
    def __init__(self):
        super(Loss,self).__init__()
        self.Loss=nn.BCEWithLogitsLoss()
    def forward(self,true_label,pre_label):
        return self.Loss(pre_label,true_label)

breast_dataset=sio.loadmat('./breastCancerWisconsinSet.mat')
def main():
    np.random.seed(123)
    #加载数据集
    dataset = sio.loadmat('./breastCancerWisconsinSet.mat')
    #将数据集划分为训练集和验证集
    train_length=int(dataset['breastCancerWisconsinTrain'].shape[0]*0.7)
    eval_length=dataset['breastCancerWisconsinTrain'].shape[0]-train_length
    #加载数据集
    dataset=sio.loadmat('./breastCancerWisconsinSet.mat')
    data_set=dataset['breastCancerWisconsinTrain']
    data_label=dataset['breastCancerWisconsinTrainLabel']//4
    np.random.seed(123)
    np.random.shuffle(data_set)
    np.random.seed(123)
    np.random.shuffle(data_label)
    t_breast_dataset=data_set[:train_length,:]
    t_breast_label=data_label[:train_length,:]
    e_breast_dataset=data_set[train_length:,:]
    e_breast_label =data_label[train_length:, :]
    train_breast_dataset=Breast_data(t_breast_dataset,t_breast_label,train_length,mode='Train')
    train_dataloader=torch.utils.data.DataLoader(train_breast_dataset,batch_size=50,shuffle=True,num_workers=1)
    eval_breast_dataset = Breast_data(e_breast_dataset,e_breast_label, eval_length,mode='eval')
    eval_dataloader = torch.utils.data.DataLoader(eval_breast_dataset, batch_size=eval_length, shuffle=True, num_workers=10)
    #前馈神经网络初始化
    estimator = Breast_net(10,[32,16,4])
    # estimator=Breast_net(10,32,1)
    estimator.cuda()
    criterion=Loss()
    optimizer=optim.Adam(estimator.parameters(),lr=0.01)
    for epoch in range(5):
        estimator.train()
        optimizer.zero_grad()
        train_loss=0
        for rep in range(200):
            for i,data in enumerate(train_dataloader,0):
                attribute,true_labels=data
                attribute,true_labels=Variable(attribute).cuda(),Variable(true_labels).cuda()
                pre_label=estimator(attribute)
                loss=criterion(pre_label,true_labels)
                train_loss += loss
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()
        print('epoch{0} train_loss is {1}'.format(epoch, train_loss))
        for i,data in enumerate(eval_dataloader,0):
            estimator.eval()
            attribute,true_labels = data
            attribute, true_labels = Variable(attribute).cuda(), Variable(true_labels).cuda()
            pre_label = estimator(attribute)
            loss = criterion(pre_label, true_labels)
            print('epoch{0} eval_loss is {1}'.format(epoch,loss) )
    #训练测试集
    test_breast_dataset=Breast_data(dataset['breastCancerWisconsinTest'],data_label=None,length=455,mode='Test')
    test_dataloader=torch.utils.data.DataLoader(test_breast_dataset,batch_size=455,shuffle=False)
    for i, data in enumerate(test_dataloader, 0):
        estimator.eval()
        attribute=data
        pre_label=estimator(attribute)
        pre_label=pre_label.view(-1,1).detach().numpy()
        for i in range(455):
            if pre_label[i]>0.5:
                pre_label[i]=4
            else:
                pre_label[i]=2
        print(pre_label)
if __name__=='__main__':
    main()




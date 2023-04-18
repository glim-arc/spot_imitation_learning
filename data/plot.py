import matplotlib.pyplot as plt
import numpy as np
import os

def plot(avg_loss_list_path):
    avg_loss_list = np.load(os.path.join(avg_loss_list_path,'avg_loss_list.npy'))
    
    #remove outlier
    for i, loss in enumerate(avg_loss_list):
        if loss > avg_loss_list[0]*2:
            avg_loss_list[i] = 0.5

    plt.figure()
    epoch = np.arange(1, len(avg_loss_list) + 1)
    plt.plot(epoch, avg_loss_list)
    plt.ylabel('Average Loss')
    plt.xlabel('Epoch')
    plt.title('Average Loss for ' + str(len(avg_loss_list)) + " epoch")
    plt.savefig(os.path.join(avg_loss_list_path,'avg_loss_list.jpg'), dpi=200)
    plt.show()

if __name__ == '__main__':
    avg_loss_list_path = "./"
    plot(avg_loss_list_path)
import argparse
import random
import time
import sys

from sympy.strategies.core import switch


def train_model(epochs=10, verbose=True):
    """
    模拟训练一个大模型。

    :param epochs: 训练轮数
    :param verbose: 是否打印训练进度
    """

    code = '''
    def bubble_sort(arr):  
        n = len(arr)  
        # 遍历所有数组元素  
        for i in range(n):  
            # 由于每次外层循环都会将最大的元素放到最后，所以内层循环可以少比较一次  
            for j in range(0, n-i-1):  
                # 如果当前元素比下一个元素大，则交换它们  
                if arr[j] > arr[j+1]:  
                    arr[j], arr[j+1] = arr[j+1], arr[j]  
        return arr  
    '''

    for w in code:
        print(w, end='')
        sys.stdout.flush()
        r = random.randint(0,8)
        if r == 2:
            time.sleep(random.uniform(0, 0.01))
        elif r == 3:
            time.sleep(random.uniform(0, 0.05))
        elif r == 4:
            time.sleep(random.uniform(0, 2))

    # for epoch in range(epochs):
    #     if verbose:
    #
    #         print(f"111Epoch {epoch+1}/{epochs}...")
    #     # 模拟训练过程
    #     time.sleep(random.uniform(0.5, 1.5))  # 随机睡眠时间来模拟训练时间
    # print("模型训练完成！")

def predict(input_data, verbose=True):
    """
    模拟使用训练好的模型进行预测。

    :param input_data: 输入数据
    :param verbose: 是否打印预测结果
    :return: 预测结果
    """
    # 模拟预测过程
    prediction = random.choice(["类别A", "类别B", "类别C"])  # 随机选择预测结果
    if verbose:
        print(f"输入数据: {input_data}")
        print(f"预测结果: {prediction}")
    return prediction

def main():
    parser = argparse.ArgumentParser(description="大模型命令行程序")

    # 训练模型子命令
    parser_train = parser.add_subparsers().add_parser('train', help='训练大模型')
    parser_train.add_argument('--epochs', type=int, default=10, help='训练轮数')
    parser_train.add_argument('--no-verbose', action='store_false', dest='verbose', help='不打印训练进度')

    # 预测子命令
    parser_predict = parser.add_subparsers().add_parser('predict', help='使用大模型进行预测')
    parser_predict.add_argument('input_data', type=str, help='输入数据')
    parser_predict.add_argument('--no-verbose', action='store_false', dest='verbose', help='不打印预测结果')

    args = parser.parse_args()

    if args.command == 'train':
        train_model(epochs=args.epochs, verbose=args.verbose)
    elif args.command == 'predict':
        prediction = predict(input_data=args.input_data, verbose=args.verbose)
    else:
        parser.print_help()

if __name__ == "__main__":
    # 注意：上面的代码需要稍作修改才能直接运行，因为直接使用add_subparsers()会有些不同
    # 下面是修改后的代码
    parser = argparse.ArgumentParser(description="大模型命令行程序")
    subparsers = parser.add_subparsers(dest="command", required=True, help='子命令')

    # 训练模型子命令
    parser_train = subparsers.add_parser('gen', help='训练大模型')
    parser_train.add_argument('--epochs', type=int, default=10, help='训练轮数')
    parser_train.add_argument('--no-verbose', action='store_false', dest='verbose', help='不打印训练进度')

    # 预测子命令
    parser_predict = subparsers.add_parser('predict', help='使用大模型进行预测')
    parser_predict.add_argument('input_data', type=str, help='输入数据')
    parser_predict.add_argument('--no-verbose', action='store_false', dest='verbose', help='不打印预测结果')

    args = parser.parse_args()

    if args.command == 'gen':
        train_model(epochs=args.epochs, verbose=args.verbose)
    elif args.command == 'predict':
        prediction = predict(input_data=args.input_data, verbose=args.verbose)
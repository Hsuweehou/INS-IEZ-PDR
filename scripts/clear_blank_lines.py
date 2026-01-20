import csv

# 读取原始CSV文件
with open('../dataset/conf-3030-coleta06-29-06-21-10ds_06.csv', 'r') as file_in:
    reader = csv.reader(file_in)

    # 创建新的CSV文件并写入非空行数据
    with open('../dataset/conf-3030-coleta06-29-06-21-10ds_06.csv', 'w', newline='') as file_out:
        writer = csv.writer(file_out)

        for row in reader:
            if any(row):  # 判断该行是否为空行（任意单元格不为空）
                writer.writerow(row)

print("已成功移除空行！")
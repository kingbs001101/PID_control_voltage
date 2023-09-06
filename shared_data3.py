# 멀티프로세스를 통해 전압 출력(step func), 전압 측정, PID 제어, csv 파일 저장 
# 4%의 offset 포함

# 필요 라이브러리를 가져온다
# DAQ 접근 
import nidaqmx  
# 그래프 그리기 
import matplotlib.pyplot as plt
# 시간 제어  
import time  
# 멀티프로세스 
import multiprocessing  
# 컴퓨터 sys 접근 
import os  
# 데이터 저장 
import csv  

# PID 게인값 입력
# 실습 시 다루는 부분 
Kp = 0.65     # 비례 제어
Ki = 0.49     # 적분 제어
Kd = 0.000001 # 미분 제어

# setpoint 입력받기(step func)(1v)
# ao1 채널이 ai1 채널과 연결되어있음 
# ao1 채널의 출력을 ai1 채널로 입력받아 관찰 
# 실습 시에는 모터 rpm 신호가 되겠다.
def output_ao1() : 
    with nidaqmx.Task() as task:
        task.ao_channels.add_ao_voltage_chan("Dev1/ao1")

        # 첫 2초 동안 0v 유지
        start_time_1 = time.time()
        while time.time() - start_time_1 <2 : 
            task.write([0.0], auto_start=True)
        
        # 2초가 흐른 뒤, 1v 출력
        while True :     
            task.write([1.0], auto_start=True)


# ao0의 채널을 PID 제어를 통해 전압 출력
def output(control_signal):
    
    with nidaqmx.Task() as task:
        task.ao_channels.add_ao_voltage_chan("Dev1/ao0")
        # 이 때 control_signal은 PID 제어법을 통해 도출 된 출력 전압 신호
        task.write([control_signal], auto_start=True)


# PID 제어로직
# 보강 필요
def pid_control(setpoint, current_value, integral_sum, prev_error):
    # 오차계산 : setpoin = ai1의 입력 전압, current-value = ao1 출력전압
    error = setpoint - current_value

    # integral_sum = 오차의 누적 값
    integral_sum += error

    # derivative = 오차 - 이전 오차
    # 실제 Ki 부분은 시간으로 나눠야 미분 오차변화율이 된다.
    derivative = error - prev_error
    # derivative = (error - prev_error) / time_interval

    # PID 제어를 통해 출력할 전압 값 계산
    control_signal = Kp * error + Ki * integral_sum + Kd * derivative

    # 루프 반복시 마다 종료 전 오차 값을 이전 오차로 할당
    prev_error = error

    # 계산결과 제공
    return control_signal, integral_sum, prev_error

# ai0,ai1 채널의 입력 전압값을 읽어와서 공유한다.(다른 프로세스에서 사용가능하게)
# PID 제어 함수로 출력 전압 계산 
def read_ai0_voltage(shared_data, lock):

    # 초기설정
    integral_sum = 0
    prev_error = 0

    # DAQ 접근
    with nidaqmx.Task() as task_ai0, nidaqmx.Task() as task_ai1:
        task_ai0.ai_channels.add_ai_voltage_chan("Dev1/ai0")
        task_ai1.ai_channels.add_ai_voltage_chan("Dev1/ai1")
        
        # 반복문 
        # ai0, ai1 전압값 읽어오기
        start_time_1 = time.time()
        while True:
            voltage_ai0 = task_ai0.read()
            voltage_ai1 = task_ai1.read()
            
            # 측정 시간 저장
            timestamp = time.time() - start_time_1 

            # PID 제어 함수를 통해서 출력 전압 계산 
            control_signal, integral_sum, prev_error = pid_control(voltage_ai1, voltage_ai0, integral_sum, prev_error)
            
            # 계산된 출력 전압으로 ai0 전압 출력 
            output(control_signal)

            # 측정시간, ai0 입력 전압, ai1 입력 전압 공유
            with lock:
                shared_data[0] = timestamp
                shared_data[1] = voltage_ai0
                shared_data[2] = voltage_ai1

            # 0.001초 대기
            time.sleep(0.001)

# 그래프 그리기 함수
def plot_ai0_data(shared_data, lock):

    # 공유 받아 올 데이터 리스트 생성 
    Data_ai0 = []
    Data_ai1 = []
    Time_stamps = [] 

    # 그래프 창 설정 
    fig, ax = plt.subplots()
    ax.set_xlabel('time')
    ax.set_ylabel('voltage')
    ax.set_title('Voltage - AI0')

    # 이하 반복문 
    start_time = time.time()
    while time.time() - start_time < 10:
        
        # 공유 받아오기
        with lock:
          
            # 데이터를 리스트에 추가
            Time_stamps.append(shared_data[0])
            Data_ai0.append(shared_data[1])
            Data_ai1.append(shared_data[2])
     
            # 그래프 지우기 (이유가 있다.)
            ax.clear() 
            
            # 그래프 그리기(x축 = 측정 시간(초), y축 = 전압 값(v))
            ax.plot(Time_stamps, Data_ai0, color='red', label='AI0')
            ax.plot(Time_stamps, Data_ai1, color='black', label='AI1')

            # offset 그래프에 표시(4%의 오차)
            ax.axhline(y=1.04, color='blue', linestyle='--',label='4% offset')
            ax.axhline(y=0.96, color='blue', linestyle='--')

            # x축을 0~10초까지 설정
            ax.set_xlim(0, 10)

            # 범례 표시
            ax.legend()

            # 그래프 일시정지
            plt.pause(0.001)

        # 그래프 창을 닫으면 반복문 탈출
        if not plt.fignum_exists(fig.number):
            break

# 실시간으로 전압 및 시간 데이터들을 File로 저장
# 그래프 프로세스와 별도로 실행되기에 훨씬 빨라서 더 많은 데이터를 저장가능
def csv_writer(shared_data, filename, lock):

    # 이하 반복문
    # 데이터들을 공유받아온다
    while True:
        timestamp = shared_data[0]
        voltage_ai0 = shared_data[1]
        voltage_ai1 = shared_data[2]

        # 4%의 offset도 함께 File내에 저장하기 위해서
        value_4th_column = 0.96
        value_5th_column = 1.04

        # 파일을 열어서 데이터들을 저장
        # 1열 : 측정시간
        # 2열 : ai0 채널 전압값
        # 3열 : ai0 채널 전압값
        # 4열 : 0.96
        # 5열 : 1.04
        with lock:
            with open(filename, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow([timestamp, voltage_ai0, voltage_ai1, value_4th_column, value_5th_column])
        
        # 과부하 방지
        time.sleep(0.001)
    
    

# 메인 스크립트 실행
if __name__ == "__main__":
    # 데이터 공유를 위해 공간 만들기
    shared_data = multiprocessing.Array('d', 3) 

    # 멀티프로세스 모듈 내 내장함수 lock
    lock = multiprocessing.Lock()

    # 저장될 File 이름 선언
    filename = 'pid_control_2_0906_0731.csv'

    # File이 저장될 경로 지정
    excel_directory = r'C:\Users\USER\Desktop\about Python\PID제어_CODE\PID_FILE'
    
    # 파일명과 경로를 한쌍으로 묶기
    full_path = os.path.join(excel_directory, filename)

    # 멀티프로세스 선언
    process_1 = multiprocessing.Process(target=read_ai0_voltage, args=(shared_data, lock))
    process_2 = multiprocessing.Process(target=plot_ai0_data, args=(shared_data, lock))
    process_3 = multiprocessing.Process(target=csv_writer, args=(shared_data, full_path, lock))
    process_4 = multiprocessing.Process(target=output_ao1, args=())
    process_5 = multiprocessing.Process(target=output, args=(0.0,))

    
    # 멀티프로세스 병렬 처리 시작
    print("*******************************\nStart multiprocessing.")
    process_1.start()
    process_2.start()
    process_3.start()
    process_4.start()
    process_5.start()

    # process_2가 종료되면 메인프로세스에 합류하도록 설정
    process_2.join()

    # process_2가 종료되면 다른 모든 프로세스도 종료되도록 설정
    if not process_2.is_alive():

        # 프로세스 종료
        process_1.terminate()
        process_3.terminate()
        process_4.terminate()
        process_5.terminate()

        # 메인 프로세스에 합류
        process_1.join()
        process_3.join()
        print("All data has been saved.")

        process_4.join()
        process_5.join()

        # 메인프로세스 제외 모든 프로세스 종료 시 구문 출력
        print("All processes have finished.")

        # 모든 채널 전압 0으로 초기화 시키기
        start_time_finish = time.time()
        while time.time() - start_time_finish < 1:
            
            with nidaqmx.Task() as task_ao0, nidaqmx.Task() as task_ao1:
                task_ao0.ao_channels.add_ao_voltage_chan("Dev1/ao0")
                task_ao1.ao_channels.add_ao_voltage_chan("Dev1/ao0")

                task_ao0.write([0.0])
                task_ao1.write([0.0]) 

        print("Reset completed.\n*******************************")

/**
* This file is part of ac^2SLAM.
*
* Copyright (C) 2021 Cheng Wang <wangcheng at stu dot xjtu dot edu dot cn> (Xi'an Jiaotong University)
* For more information see <https://github.com/SLAM-Hardware/acSLAM>
*
* ac^2SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ac^2SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ac^2SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "resize.h"

using namespace std;

ap_ufixed<16, 2> scale; // ap_ufixed类型表示无符号定点数，数据宽度为16位，整数位数为2位
ap_ufixed<16, 2> inv_scale;
ap_uint<32> width; // ap_uint类型表示无符号整数，数据宽度为32位
ap_uint<32> height;
ap_ufixed<WIDTH_BIT + 8, WIDTH_BIT> new_width; // ap_ufixed类型表示无符号定点数，数据宽度为WIDTH_BIT+8位，整数位数为WIDTH_BIT位
ap_ufixed<HEIGHT_BIT + 8, HEIGHT_BIT> new_height;
ap_uint<WIDTH_BIT> unit_num;
const ap_ufixed<16, 2> scale_1 = 1; 

void process(hls::stream<ap_axiu<INPUT_STREAM_BIT, 1, 1, 1> > &srcStream, hls::stream<ap_axiu<OUTPUT_STREAM_BIT, 1, 1, 1> > &outStream);

void process_cfg(hls::stream<ap_axiu<32, 1, 1, 1> > &cfgStream, hls::stream<ap_axiu<32, 1, 1, 1> > &cfgoutStream);

void process_scale_1(hls::stream<ap_axiu<INPUT_STREAM_BIT, 1, 1, 1> > &srcStream, hls::stream<ap_axiu<OUTPUT_STREAM_BIT, 1, 1, 1> > &outStream);

void process_input(hls::stream<ap_axiu<INPUT_STREAM_BIT, 1, 1, 1> > &srcStream, hls::stream<ap_uint<PROCESS_BIT> > &pixelData);

void process_buf(hls::stream<ap_uint<PROCESS_BIT> > &pixelData, hls::stream<ap_uint<PROCESS_BIT> > &outData);

void process_select(hls::stream<ap_uint<PROCESS_BIT> > &outData, hls::stream<ap_uint<8 + PROCESS_BIT> > &selectData);

void process_output(hls::stream<ap_uint<8 + PROCESS_BIT> > &selectData, hls::stream<ap_axiu<OUTPUT_STREAM_BIT, 1, 1, 1> > &outStream);

void resize(
    hls::stream<ap_axiu<32, 1, 1, 1> > &cfgStream, // 配置输入流，数据宽度为 32 位，包含一个用户位、一个保留位和一个 ID 位。
    hls::stream<ap_axiu<INPUT_STREAM_BIT, 1, 1, 1> > &srcStream, // 输入图像流
    hls::stream<ap_axiu<32, 1, 1, 1> > &cfgoutStream, // 配置输出流
    hls::stream<ap_axiu<OUTPUT_STREAM_BIT, 1, 1, 1> > &outStream // 输出图像流
    )
{
// HLS INTERFACE 只在顶层函数中使用，用于指定函数的输入输出接口类型，包括数据接口类型、控制接口类型等。
#pragma HLS INTERFACE ap_ctrl_none port = return // 该函数没有标准的控制信号（如启动、完成等），纯数据流处理，不需要外部控制信号来启动或停止。
#pragma HLS INTERFACE axis register both port = cfgStream // AXI-Stream 接口类型，register用于数据在传输过程中寄存器锁存，both表示输入输出都使用寄存器锁存
#pragma HLS INTERFACE axis register both port = srcStream 
#pragma HLS INTERFACE axis register both port = cfgoutStream
#pragma HLS INTERFACE axis register both port = outStream
    
    process_cfg(cfgStream, cfgoutStream); // 处理配置流
    if (scale == 1) 
        process_scale_1(srcStream, outStream); // 处理输入图像流
    else 
        process(srcStream, outStream);
}

void process_cfg(
    hls::stream<ap_axiu<32, 1, 1, 1> > &cfgStream, 
    hls::stream<ap_axiu<32, 1, 1, 1> > &cfgoutStream
    )
{
#pragma HLS INLINE off // 该函数不进行内联展开，不进行内联展开的函数将被调用时保持原样，不会展开到调用处。

    // 读取配置流数据，配置流数据包含原始图像的宽度、高度、缩放比例和缩放比例的倒数
    width = cfgStream.read().data; // 读取第1个数据：原始图像宽度
    height = cfgStream.read().data; // 读取第2个数据：原始图像高度
    scale.range(15, 0) = cfgStream.read().data.range(15, 0); // 读取第3个数据的低16位：缩放比例
    inv_scale.range(15, 0) = cfgStream.read().data.range(15, 0); // 读取第4个数据的低16位：缩放比例的倒数（反缩放比例）

    // 计算缩放后的图像宽度和高度
    ap_axiu<32, 1, 1, 1> cfgout; // 配置输出流数据类型，数据宽度为 32 位
    new_width = width / scale; // 计算缩放后的图像宽度
    new_width = my_floor<ap_ufixed<WIDTH_BIT + 8, WIDTH_BIT>, WIDTH_BIT + 8, WIDTH_BIT>(new_width); // 向下取整
    new_height = height / scale; // 计算缩放后的图像高度
    new_height = my_floor<ap_ufixed<HEIGHT_BIT + 8, HEIGHT_BIT>, HEIGHT_BIT + 8, HEIGHT_BIT>(new_height); // 向下取整

    // 将每行像素分成 unit_num 个单元，每个单元含有 16 个像素
    ap_ufixed<WIDTH_BIT + 8, WIDTH_BIT> unit_num_ufixed = width; // 计算每个单元的像素数
    unit_num_ufixed = my_ceil<ap_ufixed<WIDTH_BIT + 8, WIDTH_BIT>, WIDTH_BIT + 8, WIDTH_BIT>(unit_num_ufixed / PROCESS_NUM);// 计算每个单元的像素数并向上取整
    unit_num = unit_num_ufixed.range(WIDTH_BIT + 7, 8); // 取出每个单元的像素数的整数部分

    // 将缩放后的图像宽度和高度写入配置输出流
    cfgout.data = new_width.range(WIDTH_BIT + 7, 8); // 将缩放后的图像宽度的整数部分写入配置输出流
    cfgout.keep = 0xF; // 保持所有数据，即4个字节（cfgout的数据宽度为 32 位，32 / 8 = 4）
    cfgout.last = 0; // 非最后一个数据
    cfgoutStream.write(cfgout);
    cfgout.data = new_height.range(HEIGHT_BIT + 7, 8); // 将缩放后的图像高度的整数部分写入配置输出流
    cfgout.keep = 0xF;
    cfgout.last = 1; // 最后一个数据
    cfgoutStream.write(cfgout);
}

// 处理缩放比例为 1 的输入图像流
void process_scale_1(
    hls::stream<ap_axiu<INPUT_STREAM_BIT, 1, 1, 1> > &srcStream, // 输入图像流，数据宽度为 16 * 8 = 128 位
    hls::stream<ap_axiu<OUTPUT_STREAM_BIT, 1, 1, 1> > &outStream // 输出图像流，数据宽度为 4 * 8 = 32 位
    )
{
    ap_ufixed<PIXEL_NUM_BIT+8, PIXEL_NUM_BIT> read_tc = height*width; // 计算原始图像的像素总数
    read_tc = my_ceil<ap_ufixed<PIXEL_NUM_BIT+8, PIXEL_NUM_BIT>, PIXEL_NUM_BIT+8, PIXEL_NUM_BIT>(read_tc / INPUT_PIXEL_NUM); // 原始图像的像素数分为16个线程
    ap_uint<PIXEL_NUM_BIT> read_tc_uint = read_tc; // 每个线程像素数的整数部分
    ap_uint<PIXEL_NUM_BIT> read_ind = 0; // 读取索引
    ap_uint<8> pixel_num = 0;
    ap_uint<8> rmn_num = 0; // 剩余像素数：0-4
    ap_uint<INPUT_BIT> data = 0;
    ap_uint<OUTPUT_BIT> write_tmp = 0;
    ap_uint<PIXEL_NUM_BIT> p_cnt = 0; // 已传输像素数
    ap_axiu<OUTPUT_STREAM_BIT, 1, 1, 1> out;
    while (read_ind < read_tc_uint || rmn_num >= OUTPUT_PIXEL_NUM) // 
    {
// 优化硬件设计中的循环和数据流，以提高性能
#pragma HLS DEPENDENCE variable=rmn_num inter RAW false
#pragma HLS DEPENDENCE variable=read_ind intra RAW false
#pragma HLS PIPELINE // 用于启用循环的流水线化
        if (rmn_num < OUTPUT_PIXEL_NUM) // 读取 16 个像素的输入图像流数据，并传输其中前 4 个像素的数据
        {
            data = srcStream.read().data; // 读取输入图像流数据,数据宽度为 16 * 8 = 128 位
            if (read_ind == read_tc_uint - 1) // 判断是否为最后一个线程
                pixel_num = height * width - INPUT_PIXEL_NUM * (read_tc_uint - 1); // 最后剩余的不足 16 的像素数（为什么不对16取模？）
            else
                pixel_num = INPUT_PIXEL_NUM; // 16
            p_cnt = p_cnt + pixel_num; // 更新已经传输的像素数量
            read_ind = read_ind + 1;
            if (rmn_num + pixel_num >= OUTPUT_PIXEL_NUM) // 正常处理；如果剩余像素数加上当前线程的像素数大于等于 4
            {
                write_tmp.range(OUTPUT_BIT - 1, rmn_num * PIXEL_BIT) = data.range(OUTPUT_BIT - 1 - rmn_num * PIXEL_BIT, 0); // 传输第一组 4 个像素数据
                rmn_num = rmn_num + pixel_num - OUTPUT_PIXEL_NUM; // 更新剩余像素数
                out.data = write_tmp;
                for (ap_uint<8> i = 0; i < OUTPUT_BIT >> 3; i++)
#pragma HLS UNROLL
                    out.keep.range(i, i) = 1;
                if (p_cnt == width * height && rmn_num == 0)
                    out.last = 1;
                else
                    out.last = 0;
                outStream.write(out);
                write_tmp = 0;
                if (rmn_num > 0)
                {
                    write_tmp = data.range(pixel_num * PIXEL_BIT - 1, (pixel_num - rmn_num) * PIXEL_BIT); // 将第二组 4 像素输入图像数据，写入缓冲区
                }
            }
            else // 如果剩余像素数加上当前线程的像素数小于 4，不足以填充满输出数据
            {
                if (pixel_num > 0) // 当然仍有未传输的像素数据
                {
                    write_tmp.range((rmn_num + pixel_num) * PIXEL_BIT - 1, rmn_num * PIXEL_BIT) = data.range(pixel_num * PIXEL_BIT - 1, 0); // 传输最后剩余的不足 16 的像素
                }
                rmn_num = rmn_num + pixel_num; // 更新剩余像素数
            }
        }
        else // 传输第二、三、四个 4 字节数据
        {
            rmn_num = rmn_num - OUTPUT_PIXEL_NUM; // 更新剩余像素数
            out.data = write_tmp; 
            for (ap_uint<8> i = 0; i < OUTPUT_BIT >> 3; i++) // 输出数据的保持位，右移三位相当于除以8，即输出数据的字节数
#pragma HLS UNROLL // 循环展开
                out.keep.range(i, i) = 1; // 保持位，表示数据有效，即数据的每个字节都有效
            if (p_cnt == width * height && rmn_num == 0) // 
                out.last = 1; // 最后一个数据
            else
                out.last = 0; // 非最后一个数据
            outStream.write(out); // 传输输出数据
            write_tmp = 0;  // 输出数据清零
            if (rmn_num > 0) // 
                write_tmp = data.range(pixel_num * PIXEL_BIT - 1, (pixel_num - rmn_num) * PIXEL_BIT); 
        }
    } // while

    if (rmn_num > 0)
    {
        out.data = write_tmp;
        for (ap_uint<8> i = 0; i < OUTPUT_BIT >> 3; i++)
#pragma HLS UNROLL
            out.keep.range(i, i) = 1;
        out.last = 1;
        outStream.write(out);
    }
}

void process(
    hls::stream<ap_axiu<INPUT_STREAM_BIT, 1, 1, 1> > &srcStream, // 输入图像流
    hls::stream<ap_axiu<OUTPUT_STREAM_BIT, 1, 1, 1> > &outStream // 输出图像流
    )
{
#pragma HLS DATAFLOW // 优化支持函数或循环中的操作在前一个函数或循环尚未完成其所有操作时就开始操作

    hls::stream<ap_uint<PROCESS_BIT> > pixelData;
#pragma HLS STREAM variable = pixelData depth = 2 dim = 1

    hls::stream<ap_uint<PROCESS_BIT> > outData;
#pragma HLS STREAM variable = outData depth = 2 dim = 1

    hls::stream<ap_uint<8 + PROCESS_BIT> > selectData;
#pragma HLS STREAM variable = selectData depth = 2 dim = 1

    process_input(srcStream, pixelData);
    process_buf(pixelData, outData);
    process_select(outData, selectData);
    process_output(selectData, outStream);
#ifdef DEBUG
    for (int i = 0; i < height-1; i++)
    {
        for (int j = 0; j < 1; j++)
        {
            ap_uint<INPUT_BIT> data = outData.read();
            for (int read_ind = 0; read_ind<PROCESS_NUM; read_ind++)
                cout << data.range((read_ind+1)*PIXEL_BIT-1,read_ind*PIXEL_BIT) << " ";
        }
        cout << endl;
    }
#endif
}

void process_input(
    hls::stream<ap_axiu<INPUT_STREAM_BIT, 1, 1, 1> > &srcStream, // 输入图像流，数据宽度为 16 * 8 = 128 位
    hls::stream<ap_uint<PROCESS_BIT> > &pixelData // 128 位宽
    )
{
#pragma HLS INLINE off
    ap_uint<PIXEL_NUM_BIT> cnt = 0;
    ap_axiu<INPUT_STREAM_BIT, 1, 1, 1> dataIn;

    ap_uint<INPUT_BIT> data = 0;
    ap_uint<INPUT_BIT> prv_data = 0;
    ap_uint<11> rmn_num = 0;
    ap_uint<11> write_num = 0;
    ap_uint<PROCESS_BIT> write_tmp = 0;
    for (ap_uint<HEIGHT_BIT> i = 0; i < height; i++) // 外层循环，遍历图像每一行
    {
        for (ap_uint<WIDTH_BIT> j = 0; j < unit_num; j++) // 内层循环，处理图像每一行像素，将每行像素分成 unit_num 个单元，每个单元含有 16 个像素
        {
#pragma HLS PIPELINE
            if (j == unit_num - 1) // 判断是否为最后一个单元
                write_num = (width - PROCESS_NUM * (unit_num - 1)) * PIXEL_BIT; // 最后一个单元的像素数
            else
                write_num = PROCESS_NUM * PIXEL_BIT; // 16 * 8 = 128，每个单元含有 16 个像素

            if (rmn_num >= write_num) // 处理一个完整的单元的剩余部分数据
            {
                write_tmp = 0;
                write_tmp.range(write_num - 1, 0) = data.range(INPUT_BIT - rmn_num + write_num - 1, INPUT_BIT - rmn_num);
                rmn_num = rmn_num - write_num;
            }
            else // 处理一个完整的单元的第一部分数据
            {
                dataIn = srcStream.read(); // 读取输入图像流数据，数据宽度为 16 * 8 = 128 位，16个像素数据
                prv_data = data; // 保存上一次的数据
                data = dataIn.data; // 读取输入图像流数据，新数据
                if (rmn_num > 0)
                {
                    write_tmp = 0;
                    write_tmp.range(rmn_num - 1, 0) = prv_data.range(INPUT_BIT - 1, INPUT_BIT - rmn_num);
                    write_tmp.range(write_num - 1, rmn_num) = data.range(write_num - rmn_num - 1, 0);
                    rmn_num = INPUT_BIT - (write_num - rmn_num);
                }
                else
                {
                    write_tmp = 0;
                    write_tmp.range(write_num - 1, 0) = data.range(write_num - 1, 0);
                    rmn_num = INPUT_BIT - write_num;
                }
            }
            pixelData.write(write_tmp);
        }
    }
}

void process_buf(
    hls::stream<ap_uint<PROCESS_BIT> > &pixelData, 
    hls::stream<ap_uint<PROCESS_BIT> > &outData
    )
{
#pragma HLS INLINE off

    ap_uint<PIXEL_BIT * MERGE_NUM> image_buf[WIN_SZ][WIDTH_AFTER_MERGE];
#pragma HLS ARRAY_PARTITION variable = image_buf cyclic factor = 4 dim = 2

#pragma HLS RESOURCE variable = image_buf core = RAM_2P_BRAM

    ap_uint<PIXEL_BIT> win_buf[WIN_SZ][WIN_SZ + PROCESS_NUM - 1];
#pragma HLS ARRAY_PARTITION variable = win_buf complete dim = 0

    ap_uint<PIXEL_BIT> read_buf[PROCESS_NUM];
#pragma HLS ARRAY_PARTITION variable = read_buf complete dim = 0

    ap_uint<PIXEL_BIT> new_p_buf[PROCESS_NUM]; // new pixel buffer
#pragma HLS ARRAY_PARTITION variable = new_p_buf complete dim = 0

    ap_uint<WIN_SZ_BIT> win_ind[WIN_SZ]; // mapping between row index of image_buf and row index of win_buf
                                         // the win_ind[0] -th row in the img_buf stores the minimal row index of images data in line buffer.
#pragma HLS ARRAY_PARTITION variable = win_ind complete dim = 1

    ap_uint<11> write_num = 0;
    ap_uint<8> write_pixel_num = 0;

    //initialization
    for (ap_uint<WIN_SZ_BIT> i = 0; i < WIN_SZ; i++)
#pragma HLS UNROLL
        win_ind[i] = i;

    for (ap_uint<HEIGHT_BIT> row_ind = 0; row_ind < WIN_SZ - 1; row_ind++)
        for (ap_uint<WIDTH_BIT> read_ind = 0; read_ind < unit_num; read_ind++)
        {
#pragma HLS PIPELINE
            ap_uint<PROCESS_BIT> data = pixelData.read();
            for (ap_uint<8> i = 0; i < PROCESS_NUM; i++)
#pragma HLS UNROLL
                read_buf[i] = data.range((i+1)*PIXEL_BIT-1, i*PIXEL_BIT);

            if (read_ind == unit_num - 1)
            {
                write_num = (width - PROCESS_NUM * (unit_num - 1)) * PIXEL_BIT;
                write_pixel_num = width - PROCESS_NUM * (unit_num - 1);
            }
            else
            {
                write_num = PROCESS_NUM * PIXEL_BIT;
                write_pixel_num = PROCESS_NUM;
            }

            for (ap_uint<8> col_ind = 0; col_ind < PROCESS_NUM / MERGE_NUM; col_ind++)
            {
#pragma HLS UNROLL
                ap_uint<PIXEL_BIT * MERGE_NUM> merge_tmp = 0;
                for (ap_uint<LOG_2_MERGE_NUM> pixel_ind = 0; pixel_ind < MERGE_NUM; pixel_ind++)
                {
#pragma HLS UNROLL
                    if (pixel_ind + col_ind * MERGE_NUM < write_pixel_num)
                        merge_tmp.range((pixel_ind+1)*PIXEL_BIT-1, pixel_ind*PIXEL_BIT) = read_buf[pixel_ind + col_ind * MERGE_NUM];
                }
                image_buf[row_ind][col_ind + read_ind * PROCESS_NUM / MERGE_NUM] = merge_tmp;
            }
        }

Row_Loop:
    for (ap_uint<HEIGHT_BIT> row_ind = 0; row_ind < height - (WIN_SZ - 1); row_ind++)
    {
        for (ap_uint<WIDTH_BIT> read_ind = 0; read_ind < unit_num; read_ind++)
        {
#pragma HLS PIPELINE
            ap_uint<PROCESS_BIT> data = pixelData.read();
            for (ap_uint<8> i = 0; i < PROCESS_NUM; i++)
#pragma HLS UNROLL
                read_buf[i] = data.range((i+1)*PIXEL_BIT-1, i*PIXEL_BIT);

            if (read_ind == unit_num - 1)
            {
                write_num = (width - PROCESS_NUM * (unit_num - 1)) * PIXEL_BIT;
                write_pixel_num = width - PROCESS_NUM * (unit_num - 1);
            }
            else
            {
                write_num = PROCESS_NUM * PIXEL_BIT;
                write_pixel_num = PROCESS_NUM;
            }

            for (ap_uint<8> col_ind = 0; col_ind < PROCESS_NUM / MERGE_NUM; col_ind++)
            {
#pragma HLS UNROLL
                ap_uint<PIXEL_BIT * MERGE_NUM> merge_tmp = 0;
                for (ap_uint<LOG_2_MERGE_NUM> pixel_ind = 0; pixel_ind < MERGE_NUM; pixel_ind++)
                {
#pragma HLS UNROLL
                    if (pixel_ind + col_ind * MERGE_NUM < write_pixel_num)
                    {
                        merge_tmp.range((pixel_ind+1)*PIXEL_BIT-1, pixel_ind*PIXEL_BIT) = read_buf[pixel_ind + col_ind * MERGE_NUM];
                        win_buf[WIN_SZ - 1][col_ind * MERGE_NUM + pixel_ind + WIN_SZ - 1] = read_buf[col_ind * MERGE_NUM + pixel_ind];
                    }
                }
                image_buf[win_ind[WIN_SZ - 1]][col_ind + read_ind * PROCESS_NUM / MERGE_NUM] = merge_tmp;
            }

#ifdef DEBUG
            for (int i = 0; i < WIN_SZ; i++)
            {
                for (int j = 0; j < ceil((double)width / MERGE_NUM); j++)
                {
                    ap_uint<MERGE_NUM * PIXEL_BIT> cout_tmp =image_buf[i][j];
                    for (int k = 0; k < MERGE_NUM; k++)
                    {
                        cout << cout_tmp.range(7, 0) << " ";
                        cout_tmp = cout_tmp >> PIXEL_BIT;
                    }
                }
                cout << endl;
            }
            cout << "-----------------image_buf------------------" << endl;
#endif

            for (ap_uint<WIN_SZ_BIT> i = 0; i < WIN_SZ - 1; i++)
            {
#pragma HLS UNROLL
                for (ap_uint<8> col_ind = 0; col_ind < PROCESS_NUM / MERGE_NUM; col_ind++)
                {
#pragma HLS UNROLL
                    ap_uint<PIXEL_BIT * MERGE_NUM> read_tmp = image_buf[win_ind[i]][col_ind + read_ind * PROCESS_NUM / MERGE_NUM];
                    for (ap_uint<LOG_2_MERGE_NUM> pixel_ind = 0; pixel_ind < MERGE_NUM; pixel_ind++)
                    {
#pragma HLS UNROLL
                        win_buf[i][col_ind * MERGE_NUM + pixel_ind + WIN_SZ - 1] = read_tmp.range((pixel_ind+1)*PIXEL_BIT-1, pixel_ind*PIXEL_BIT);
                    }
                }
            }

#ifdef DEBUG
            for (int i = 0; i < WIN_SZ; i++)
            {
                for (int j = 0; j < WIN_SZ + PROCESS_NUM - 1; j++)
                    cout << win_buf[i][j] << " ";
                cout << endl;
            }
            cout << "-----------------win_buf------------------" << endl;
#endif

            //compute pixel
            for (ap_uint<WIDTH_BIT> col_ind = 0; col_ind < PROCESS_NUM; col_ind++)
            {
#pragma HLS UNROLL
                ap_uint<WIDTH_BIT> col_ind_tmp = col_ind + read_ind * PROCESS_NUM;

                ap_ufixed<HEIGHT_BIT + 8, HEIGHT_BIT> row_tmp = row_ind;
                ap_ufixed<WIDTH_BIT + 8, WIDTH_BIT> col_tmp = col_ind_tmp - 1;

                row_tmp = my_ceil<ap_ufixed<HEIGHT_BIT + 8, HEIGHT_BIT>, HEIGHT_BIT + 8, HEIGHT_BIT>(row_tmp * inv_scale) * scale;
                col_tmp = my_ceil<ap_ufixed<WIDTH_BIT + 8, WIDTH_BIT>, WIDTH_BIT + 8, WIDTH_BIT>(col_tmp * inv_scale) * scale;
                
                ap_ufixed<PIXEL_BIT + 4, PIXEL_BIT + 2> new_p = win_buf[0][col_ind] * ((row_ind + 1) - row_tmp) * (col_ind_tmp - col_tmp) +
                                                                win_buf[0][col_ind + 1] * ((row_ind + 1) - row_tmp) * (col_tmp - (col_ind_tmp - 1)) +
                                                                win_buf[1][col_ind] * (row_tmp - row_ind) * (col_ind_tmp - col_tmp) +
                                                                win_buf[1][col_ind + 1] * (row_tmp - row_ind) * (col_tmp - (col_ind_tmp - 1));

                if (new_p > MAX_PIXEL_VAL)
                    new_p = MAX_PIXEL_VAL;
                ap_uint<PIXEL_BIT> new_p_out = my_round<ap_ufixed<PIXEL_BIT + 4, PIXEL_BIT + 2>, PIXEL_BIT + 4, PIXEL_BIT + 2>(new_p);
                new_p_buf[col_ind] = new_p_out;
            }

#ifdef DEBUG
            for (int i = 0; i < PROCESS_NUM; i++)
            {
                cout << new_p_buf[i] << " ";
            }
            cout << endl
                 << "-----------------new_p_buf------------------" << endl;
#endif

            //ouput
            ap_uint<INPUT_BIT> out_data = 0;
            for (ap_uint<10> i = 0; i < PROCESS_NUM; i++)
#pragma HLS UNROLL
                out_data.range((i + 1) * PIXEL_BIT - 1, i * PIXEL_BIT) = new_p_buf[i];
            outData.write(out_data);

            //move win_buf
            for (ap_uint<WIN_SZ_BIT> i = 0; i < WIN_SZ; i++)
#pragma HLS UNROLL
                for (ap_uint<WIDTH_BIT> j = 0; j < WIN_SZ - 1; j++)
#pragma HLS UNROLL
                    win_buf[i][j] = win_buf[i][j + PROCESS_NUM];
        }

        ap_uint<WIN_SZ_BIT> win_ind_tmp = win_ind[0];
        for (ap_uint<WIN_SZ_BIT> i = 0; i < WIN_SZ - 1; i++)
#pragma HLS UNROLL
            win_ind[i] = win_ind[i + 1];
        win_ind[WIN_SZ - 1] = win_ind_tmp;
    }
}

void process_select(hls::stream<ap_uint<PROCESS_BIT> > &outData, hls::stream<ap_uint<8 + PROCESS_BIT> > &selectData)
{
#pragma HLS INLINE off

    ap_uint<PIXEL_BIT> image_buf[PROCESS_NUM];
#pragma HLS ARRAY_PARTITION variable = image_buf complete dim = 1

    ap_uint<PIXEL_BIT> input_buf[INPUT_PIXEL_NUM];
#pragma HLS ARRAY_PARTITION variable = input_buf complete dim = 1

    for (ap_uint<WIDTH_BIT> col_ind = 0; col_ind < PROCESS_NUM; col_ind++)
#pragma HLS UNROLL
        image_buf[col_ind] = 0;

    for (ap_uint<HEIGHT_BIT> row_ind = 0; row_ind < height - 1; row_ind++)
    {
        ap_ufixed<HEIGHT_BIT + 8, HEIGHT_BIT> row_ind_up = row_ind * inv_scale;
        ap_ufixed<HEIGHT_BIT + 8, HEIGHT_BIT> row_ind_down = (row_ind + 1) * inv_scale;
        row_ind_up = my_ceil<ap_ufixed<HEIGHT_BIT + 8, HEIGHT_BIT>, HEIGHT_BIT + 8, HEIGHT_BIT>(row_ind_up);
        row_ind_down = my_ceil<ap_ufixed<HEIGHT_BIT + 8, HEIGHT_BIT>, HEIGHT_BIT + 8, HEIGHT_BIT>(row_ind_down);

        for (ap_uint<WIDTH_BIT> read_ind = 0; read_ind < unit_num; read_ind++)
        {
#pragma HLS PIPELINE
            ap_uint<PROCESS_BIT> data = outData.read();
            for (ap_uint<8> i = 0; i < INPUT_PIXEL_NUM; i++)
#pragma HLS UNROLL
                input_buf[i] = data.range((i + 1) * PIXEL_BIT - 1, i * PIXEL_BIT);
            if (row_ind_up != row_ind_down && row_ind_down <= new_height)
            {
                ap_uint<WIDTH_BIT> min_col_ind_tmp = read_ind * PROCESS_NUM;
                ap_uint<WIDTH_BIT> max_col_ind_tmp = PROCESS_NUM - 1 + read_ind * PROCESS_NUM;
                ap_ufixed<WIDTH_BIT + 8, WIDTH_BIT> min_col_ind = (min_col_ind_tmp - 1) * inv_scale;
                ap_ufixed<WIDTH_BIT + 8, WIDTH_BIT> max_col_ind = max_col_ind_tmp * inv_scale;
                min_col_ind = my_ceil<ap_ufixed<WIDTH_BIT + 8, WIDTH_BIT>, WIDTH_BIT + 8, WIDTH_BIT>(min_col_ind);
                max_col_ind = my_ceil<ap_ufixed<WIDTH_BIT + 8, WIDTH_BIT>, WIDTH_BIT + 8, WIDTH_BIT>(max_col_ind);
                if (max_col_ind > new_width)
                    max_col_ind = new_width;
                for (ap_uint<WIDTH_BIT> col_ind = 0; col_ind < PROCESS_NUM; col_ind++)
                {
#pragma HLS UNROLL
                    ap_uint<WIDTH_BIT> col_ind_left = col_ind + min_col_ind;
                    ap_ufixed<WIDTH_BIT + 8, WIDTH_BIT> col_ind_tmp = col_ind_left * scale;
                    ap_uint<WIDTH_BIT> col_ind_tmp_uint = col_ind_tmp;
                    if (col_ind_left < max_col_ind){
                        if (col_ind_tmp_uint + 1 >= read_ind * PROCESS_NUM)
                            col_ind_tmp_uint = col_ind_tmp_uint + 1 - read_ind * PROCESS_NUM;
                        else
                            col_ind_tmp_uint = 0;
                    }
                    else
                        col_ind_tmp_uint = 0;
                    image_buf[col_ind] = input_buf[col_ind_tmp_uint];
                }
#ifdef DEBUG
                cout << row_ind << endl;
                cout << row_ind_up << " " << row_ind_down << endl;
                for (int i = 0; i < PROCESS_NUM; i++)
                    cout << image_buf[i] << " ";
                cout << endl
                     << "----------------------------------new_image_buf--------------------------------" << endl;
#endif

                ap_uint<8 + PROCESS_BIT> write_tmp = 0;
                for (ap_uint<8> i = 0; i < PROCESS_NUM; i++)
                {
#pragma HLS UNROLL
                    write_tmp.range((i + 1) * PIXEL_BIT - 1, i * PIXEL_BIT) = image_buf[i];
                }
                write_tmp.range(7 + PROCESS_BIT, PROCESS_BIT) = max_col_ind - min_col_ind;
                selectData.write(write_tmp);
            }
        }
    }
}

void process_output(hls::stream<ap_uint<8 + PROCESS_BIT> > &selectData, hls::stream<ap_axiu<OUTPUT_STREAM_BIT, 1, 1, 1> > &outStream)
{
    ap_uint<PROCESS_BIT> data = 0;
    ap_uint<OUTPUT_BIT> write_tmp = 0;
    ap_axiu<OUTPUT_STREAM_BIT, 1, 1, 1> out;
    ap_uint<8> pixel_num = 0;
    ap_uint<8> rmn_num = 0;
    ap_uint<PIXEL_NUM_BIT> p_cnt = 0;
    ap_uint<8 + PROCESS_BIT> s_data = 0;
    for (ap_uint<HEIGHT_BIT> row_ind = 0; row_ind < new_height; row_ind++)
    {
        ap_uint<WIDTH_BIT> read_ind = 0;
        while (read_ind < unit_num || rmn_num >= OUTPUT_PIXEL_NUM)
        {
#pragma HLS PIPELINE

            if (rmn_num >= OUTPUT_PIXEL_NUM)
            {
                rmn_num = rmn_num - OUTPUT_PIXEL_NUM;
                out.data = write_tmp;
                for (ap_uint<8> i = 0; i < OUTPUT_BIT >> 3; i++)
#pragma HLS UNROLL
                    out.keep.range(i, i) = 1;
                if (p_cnt == new_width * new_height && rmn_num == 0)
                    out.last = 1;
                else
                    out.last = 0;
                outStream.write(out);
                write_tmp = 0;
                if (rmn_num > 0)
                    write_tmp = data.range(pixel_num * PIXEL_BIT - 1, (pixel_num - rmn_num) * PIXEL_BIT);
            }
            else
            {
                s_data = selectData.read();
                data = s_data.range(PROCESS_BIT - 1, 0);
                pixel_num = s_data.range(7 + PROCESS_BIT, PROCESS_BIT);
                p_cnt = p_cnt + pixel_num;
                read_ind = read_ind + 1;
                if (rmn_num + pixel_num >= OUTPUT_PIXEL_NUM)
                {
                    write_tmp.range(OUTPUT_BIT - 1, rmn_num * PIXEL_BIT) = data.range(OUTPUT_BIT - 1 - rmn_num * PIXEL_BIT, 0);
                    rmn_num = rmn_num + pixel_num - OUTPUT_PIXEL_NUM;
                    out.data = write_tmp;
                    for (ap_uint<8> i = 0; i < OUTPUT_BIT >> 3; i++)
#pragma HLS UNROLL
                        out.keep.range(i, i) = 1;
                    if (p_cnt == new_width * new_height && rmn_num == 0)
                        out.last = 1;
                    else
                        out.last = 0;
                    outStream.write(out);
                    write_tmp = 0;
                    if (rmn_num > 0)
                        write_tmp = data.range(pixel_num * PIXEL_BIT - 1, (pixel_num - rmn_num) * PIXEL_BIT);
                }
                else
                {
                    if (pixel_num > 0)
                        write_tmp.range((rmn_num + pixel_num) * PIXEL_BIT - 1, rmn_num * PIXEL_BIT) = data.range(pixel_num * PIXEL_BIT - 1, 0);
                    rmn_num = rmn_num + pixel_num;
                }
            }
        }
    }
    if (rmn_num > 0)
    {
        out.data = write_tmp;
        for (ap_uint<8> i = 0; i < OUTPUT_BIT >> 3; i++)
#pragma HLS UNROLL
            out.keep.range(i, i) = 1;
        out.last = 1;
        outStream.write(out);
    }
}

/**
 * @file PlotCalculationProcess.cpp
 * @brief 計算過程のプロットに関する実装
 *
 * @author 福司 謙一郎
 * @date 2009
 */

#include <vector>
#include <iostream>
#include <fstream>
#include "../humantracking.h"
#include "track.h"

using namespace std;

void OutputProcess( TIME_MICRO_SEC timeBegin
                     , TIME_MICRO_SEC timeEnd
                     , TIME_MICRO_SEC offset
                     , SamplerPosXYTVID* pSampler
                     , unsigned int nSample
                     , vector<TrajectoryElement>* pTrajectoryElementVec
                     , FILE* pipeGnuplot
                     , double stdDev
                     , const PARAM_PLOT* pPlotParam )
{
    timeBegin -= offset;
    timeEnd -= offset;
    double timeBegin_d = (double)timeBegin / 1.0e6;
    double timeEnd_d = (double)timeEnd / 1.0e6;
    string strSampleFile, strTrajectoriesFile, strScriptFile;

    // サンプルの出力
    {
        ofstream ofs;
        ostringstream oss;

        // サンプラーからサンプルを取得
        vector<PosXYTVID> posVec( nSample );
        if( posVec.size() > 0 ) {
            pSampler->Execute( (PosXYTVID*)&(*posVec.begin()), nSample );
            if( stdDev > 0.0 ) {
                Gaussian( &posVec, stdDev );
            }
        }

        // ファイルを開きサンプルのデータを出力する
#ifdef WINDOWS_OS
        oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
#endif
#ifdef LINUX_OS
	oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
#endif
	oss << "samples_" << timeBegin << ".csv";
        strSampleFile = oss.str();
        ofs.open( strSampleFile.c_str() );
        ofs << "x, z, t" << endl;
        vector<PosXYTVID>::iterator itPos = posVec.begin();
        for( ; itPos != posVec.end(); ++itPos ) {
            ofs << itPos->x << ", " << itPos->y << ", " << (double)( itPos->t - offset ) / 1.0e6 << endl;
        }
        ofs.flush();
        ofs.close();
    }

    // 軌跡の出力
    {
        ofstream ofs;
        ostringstream oss;

#ifdef WINDOWS_OS
        oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
#endif
#ifdef LINUX_OS
	oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
#endif
        oss << "trajectories_" << timeBegin << ".csv";
        strTrajectoriesFile = oss.str();
        ofs.open( strTrajectoriesFile.c_str() );
        CTrajectory trajectory;
        trajectory.assign( pTrajectoryElementVec->begin(), pTrajectoryElementVec->end() );
        trajectory.Clip( timeBegin + offset, timeEnd + offset );
        CTrajectory::iterator itTrjElement;
        for( itTrjElement = trajectory.begin(); itTrjElement != trajectory.end(); ++itTrjElement ) {
            TrajectoryElement::iterator itPos;
            for( itPos = itTrjElement->begin(); itPos != itTrjElement->end(); ++itPos ) {
                ofs << itPos->x << ", " << itPos->y << ", " << (double)( itPos->t - offset ) / 1.0e6 << endl;
            }
            ofs << endl;
        }

        ofs.flush();
        ofs.close();
    }

    // GNUPLOTのスクリプト
    {
        ofstream ofs;
        ostringstream oss;
#ifdef WINDOWS_OS
        oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_trajectories\\";
#endif
#ifdef LINUX_OS
	oss << "/home/fukushi/project/HumanTracking/bin/tmp_trajectories/";
#endif
        oss << "plot_trajectories_" << timeBegin << ".plt";
        strScriptFile = oss.str();
        ofs.open( strScriptFile.c_str() );
        ofs << "set nokey" << endl;
        ofs << "set xrange[" << pPlotParam->rangeLeft << ":" << pPlotParam->rangeRight << "]" << endl;
        ofs << "set yrange[" << timeBegin_d << ":" << timeEnd_d << "]" << endl;
        ofs << "set zrange[" << pPlotParam->rangeBottom << ":" << pPlotParam->rangeTop << "]" << endl;
        ofs << "set ticslevel 0" << endl;
        ofs << "splot '" << strSampleFile << "' using 1:3:2 with points,\\" << endl;
        ofs << "'" << strTrajectoriesFile << "' using 1:3:2 with lines" << endl;
        ofs.flush();
        ofs.close();
    }
/*
    // GNUPLOTで表示
    if( pipeGnuplot ) {
        ostringstream oss;

        oss << "load '" << strScriptFile << "'" << endl;
        fputs( oss.str().c_str(), pipeGnuplot );
        fflush( pipeGnuplot );
    }
*/
}

void OutputProcess( TIME_MICRO_SEC timeBegin
                     , TIME_MICRO_SEC timeEnd
                     , TIME_MICRO_SEC offset
                     , SamplerPosXYTVID* pSampler
                     , unsigned int nSample
                     , vector<CTrajectory>* pTrajectories
                     , string strDir
                     , string strExtention
                     , FILE* pipeGnuplot
                     , const PARAM_PLOT* pPlotParam )
{
    timeBegin -= offset;
    timeEnd -= offset;
    double timeBegin_d = (double)timeBegin / 1.0e6;
    double timeEnd_d = (double)timeEnd / 1.0e6;
    string strSampleFile, strScriptFile;
    vector<string> strTrajectoriesFile;

    // サンプルの出力
    {
        ofstream ofs;
        ostringstream oss;

        // サンプラーからサンプルを取得
        vector<PosXYTVID> posVec( nSample );
        if( posVec.size() > 0 ) {
            pSampler->Execute( (PosXYTVID*)&(*posVec.begin()), nSample );
        }

        // ファイルを開きサンプルのデータを出力する
        //oss << "./tmp_clustering/samples_" << timeBegin << ".csv";
        oss << strDir << "samples_" << timeBegin << ".csv";
        strSampleFile = oss.str();
        ofs.open( strSampleFile.c_str() );
        ofs << "x, z, t" << endl;
        vector<PosXYTVID>::iterator itPos = posVec.begin();
        for( ; itPos != posVec.end(); ++itPos ) {
            ofs << itPos->x << ", " << itPos->y << ", " << (double)( itPos->t - offset ) / 1.0e6 << endl;
        }
        ofs.flush();
        ofs.close();
    }

    // 軌跡の出力
    {
        vector<CTrajectory>::iterator itTrajectory;
        unsigned int i = 0;
        for( itTrajectory = pTrajectories->begin(); itTrajectory != pTrajectories->end(); ++itTrajectory, ++i ) {
            ofstream ofs;
            ostringstream oss;
            //oss << "./tmp_clustering/trajectories_class_" << i << "_" << timeBegin << ".csv";
            oss << strDir << "trajectories_class_" << i << "_" << timeBegin << "_" << strExtention << ".csv";
            strTrajectoriesFile.push_back( oss.str() );
            ofs.open( oss.str().c_str() );
            CTrajectory::iterator itTrjElement;
            for( itTrjElement = itTrajectory->begin(); itTrjElement != itTrajectory->end(); ++itTrjElement ) {
                TrajectoryElement::iterator itPos;
                for( itPos = itTrjElement->begin(); itPos != itTrjElement->end(); ++itPos ) {
                    ofs << itPos->x << ", " << itPos->y << ", " << (double)( itPos->t - offset ) / 1.0e6 << endl;
                }
                ofs << endl;
            }

            ofs.flush();
            ofs.close();
        }
    }

    // 外部プログラムで実験等に使用するための軌跡ファイル出力
    {
        ofstream ofs;
        ostringstream oss;

        //oss << "./tmp_clustering/" << timeBegin << ".trj";
        oss << strDir <<  timeBegin << "_" << strExtention << ".trj";
        ofs.open( oss.str().c_str() );

        vector<CTrajectory>::iterator itTrajectory;
        unsigned int i = 0;
        for( itTrajectory = pTrajectories->begin(); itTrajectory != pTrajectories->end(); ++itTrajectory, ++i ) {
            CTrajectory trajectory;
            itTrajectory->Integrate( &trajectory );
            trajectory.Clip( timeBegin + offset, timeEnd + offset );
            CTrajectory::iterator itTrjElement;
            for( itTrjElement = trajectory.begin(); itTrjElement != trajectory.end(); ++itTrjElement ) {
                TrajectoryElement::iterator itPos;
                for( itPos = itTrjElement->begin(); itPos != itTrjElement->end(); ++itPos ) {
                    ofs << itPos->t - offset - timeBegin << " " << itPos->x << " " << itPos->y << endl;
                }
                ofs << endl;
            }
        }

        ofs.flush();
        ofs.close();
    }

    // GNUPLOTのスクリプト
    {
        ofstream ofs;
        ostringstream oss;

        //oss << "./tmp_clustering/plot_trajectories_clustered" << timeBegin << ".plt";
        oss << strDir << "plot_trajectories_clustered" << timeBegin << "_" << strExtention << ".plt";
        strScriptFile = oss.str();
        ofs.open( strScriptFile.c_str() );
        ofs << "set nokey" << endl;
        ofs << "set xrange[" << pPlotParam->rangeLeft << ":" << pPlotParam->rangeRight << "]" << endl;
        ofs << "set yrange[" << timeBegin_d << ":" << timeEnd_d << "]" << endl;
        ofs << "set zrange[" << pPlotParam->rangeBottom << ":" << pPlotParam->rangeTop << "]" << endl;
        ofs << "set ticslevel 0" << endl;
        ofs << "splot '" << strSampleFile << "' using 1:3:2 with points,\\" << endl;
        vector<string>::iterator itStrTrjFile;
        for( itStrTrjFile = strTrajectoriesFile.begin(); itStrTrjFile != strTrajectoriesFile.end(); ++itStrTrjFile ) {
            ofs << "'" << *itStrTrjFile << "' using 1:3:2 with lines";
            if( itStrTrjFile + 1 != strTrajectoriesFile.end() ) {
                ofs << ",\\";
            }
            ofs << endl;
        }
        ofs.flush();
        ofs.close();
    }
/*
    // GNUPLOTで表示
    if( pipeGnuplot ) {
        ostringstream oss;

        oss << "load '" << strScriptFile << "'" << endl;
        fputs( oss.str().c_str(), pipeGnuplot );
        fflush( pipeGnuplot );
    }
*/
}


void OutputProcess( TIME_MICRO_SEC timeBegin
                     , TIME_MICRO_SEC timeEnd
                     , TIME_MICRO_SEC offset
                     , SamplerPosXYTVID* pSampler
                     , unsigned int nSample
                     , map<int,CTrajectory>* pResult
                     , FILE* pipeGnuplot
                     , const PARAM_PLOT* pPlotParam )
{
    timeBegin -= offset;
    timeEnd -= offset;
    double timeBegin_d = (double)timeBegin / 1.0e6;
    double timeEnd_d = (double)timeEnd / 1.0e6;
    string strSampleFile, strScriptFile;
    vector<string> strTrajectoriesFile;

    // サンプルの出力
    {
        ofstream ofs;
        ostringstream oss;

        // サンプラーからサンプルを取得
        vector<PosXYTVID> posVec( nSample );
        if( posVec.size() > 0 ) {
            pSampler->Execute( (PosXYTVID*)&(*posVec.begin()), nSample );
        }

        // ファイルを開きサンプルのデータを出力する
#ifdef WINDOWS_OS
        oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_renovate\\";
#endif
#ifdef LINUX_OS
	oss << "/home/fukushi/project/HumanTracking/bin/tmp_renovate/";
#endif
        oss << "samples_" << timeBegin << ".csv";
        strSampleFile = oss.str();
        ofs.open( strSampleFile.c_str() );
        ofs << "x, z, t" << endl;
        vector<PosXYTVID>::iterator itPos = posVec.begin();
        for( ; itPos != posVec.end(); ++itPos ) {
            ofs << itPos->x << ", " << itPos->y << ", " << (double)( itPos->t - offset ) / 1.0e6 << endl;
        }
        ofs.flush();
        ofs.close();
    }

    // 軌跡の出力
    {
        map<int,CTrajectory>::iterator itResult;
        for( itResult = pResult->begin(); itResult != pResult->end(); ++itResult ) {
            ofstream ofs;
            ostringstream oss;
#ifdef WINDOWS_OS
            oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_renovate\\";
#endif
#ifdef LINUX_OS
            oss << "/home/fukushi/project/HumanTracking/bin/tmp_renovate/";
#endif
            oss << "trajectories_result_" << itResult->first << "_" << timeBegin << ".csv";
            strTrajectoriesFile.push_back( oss.str() );
            ofs.open( oss.str().c_str() );
            CTrajectory trajectory;
            itResult->second.Integrate( &trajectory );
            trajectory.Clip( timeBegin + offset, timeEnd + offset );
            CTrajectory::iterator itTrjElement;
            for( itTrjElement = trajectory.begin(); itTrjElement != trajectory.end(); ++itTrjElement ) {
                TrajectoryElement::iterator itPos;
                for( itPos = itTrjElement->begin(); itPos != itTrjElement->end(); ++itPos ) {
                    ofs << itPos->x << ", " << itPos->y << ", " << (double)( itPos->t - offset ) / 1.0e6 << endl;
                }
                ofs << endl;
            }

            ofs.flush();
            ofs.close();
        }
    }

    // GNUPLOTのスクリプト
    {
        ofstream ofs;
        ostringstream oss;

#ifdef WINDOWS_OS
        oss << "C:\\Users\\fukushi\\Documents\\project\\HumanTracking\\bin\\tmp_renovate\\";
#endif
#ifdef LINUX_OS
	oss << "/home/fukushi/project/HumanTracking/bin/tmp_renovate/";
#endif
        oss << "plot_trajectories_result" << timeBegin << ".plt";
        strScriptFile = oss.str();
        ofs.open( strScriptFile.c_str() );
        ofs << "set nokey" << endl;
        ofs << "set xrange[" << pPlotParam->rangeLeft << ":" << pPlotParam->rangeRight << "]" << endl;
        ofs << "set yrange[" << timeBegin_d << ":" << timeEnd_d << "]" << endl;
        ofs << "set zrange[" << pPlotParam->rangeBottom << ":" << pPlotParam->rangeTop << "]" << endl;
        ofs << "set ticslevel 0" << endl;
        ofs << "splot '" << strSampleFile << "' using 1:3:2 with points,\\" << endl;
        vector<string>::iterator itStrTrjFile;
        for( itStrTrjFile = strTrajectoriesFile.begin(); itStrTrjFile != strTrajectoriesFile.end(); ++itStrTrjFile ) {
            ofs << "'" << *itStrTrjFile << "' using 1:3:2 with lines";
            if( itStrTrjFile + 1 != strTrajectoriesFile.end() ) {
                ofs << ",\\";
            }
            ofs << endl;
        }
        ofs.flush();
        ofs.close();
    }
/*
    // GNUPLOTで表示
    if( pipeGnuplot ) {
        ostringstream oss;

        oss << "load '" << strScriptFile << "'" << endl;
        fputs( oss.str().c_str(), pipeGnuplot );
        fflush( pipeGnuplot );
    }
*/
}

void OutputProcess( TIME_MICRO_SEC timeBegin
                     , TIME_MICRO_SEC timeEnd
                     , TIME_MICRO_SEC offset
                     , vector<int>& combination
                     , int idxCombination
                     , vector<CTrajectory>& trajectories
                     , string strDir
                     , string strExtention
                     , const PARAM_PLOT* pPlotParam )
{
    timeBegin -= offset;
    timeEnd -= offset;
    double timeBegin_d = (double)timeBegin / 1.0e6;
    double timeEnd_d = (double)timeEnd / 1.0e6;
    string strSampleFile;
    {
        ostringstream oss;
        oss << strDir << "samples_" << timeBegin << ".csv";
        strSampleFile = oss.str();
    }

    // GNUPLOTのスクリプト
    {
        ofstream ofs;
        ostringstream oss;

        oss << strDir << "plot_combination" << idxCombination << "_" << timeBegin << "_" << strExtention << ".plt";
        string strScriptFile = oss.str();
        ofs.open( strScriptFile.c_str() );
        ofs << "set nokey" << endl;
        ofs << "set xrange[" << pPlotParam->rangeLeft << ":" << pPlotParam->rangeRight << "]" << endl;
        ofs << "set yrange[" << timeBegin_d << ":" << timeEnd_d << "]" << endl;
        ofs << "set zrange[" << pPlotParam->rangeBottom << ":" << pPlotParam->rangeTop << "]" << endl;
        ofs << "set ticslevel 0" << endl;
        ofs << "splot '" << strSampleFile << "' using 1:3:2 with points,\\" << endl;
        for( int i = 0; i < combination.size(); ++i ) {
            ostringstream oss;
            oss << strDir << "trajectories_class_" << combination[ i ] << "_" << timeBegin << "_" << strExtention << ".csv";
            
            ofs << "'" << oss.str() << "' using 1:3:2 with lines";
            if( i != combination.size() - 1 ) {
                ofs << ",\\";
            }
            ofs << endl;
        }
        ofs.flush();
        ofs.close();
    }
}


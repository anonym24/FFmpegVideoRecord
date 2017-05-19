package com.example.ffmpegvideorecord;

import android.app.Activity;
import android.content.Context;
import android.hardware.Camera;
import android.media.AudioFormat;
import android.media.AudioRecord;
import android.media.MediaRecorder;
import android.view.View;
import android.widget.FrameLayout;
import org.bytedeco.javacpp.avcodec;
import org.bytedeco.javacv.FFmpegFrameRecorder;
import org.bytedeco.javacv.Frame;

import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ShortBuffer;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Stack;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.LinkedBlockingQueue;

import static java.lang.Thread.State.WAITING;

public class FFmpegVideoRecord {

    private int mCameraId;
    private FFmpegFrameRecorder mFrameRecorder;
    private VideoRecordThread mVideoRecordThread;
    private AudioRecordRunnable audioRecordRunnable;
    private volatile boolean mRecording = false;
    private LinkedBlockingQueue<FrameToRecord> mFrameToRecordQueue;
    private LinkedBlockingQueue<FrameToRecord> mRecycledFrameQueue;
    private int mFrameToRecordCount;
    private long mTotalProcessFrameTime;
    private Stack<RecordFragment> mRecordFragments;

    private int sampleAudioRateInHz = 44100;
    private int previewWidth;
    private int previewHeight;
    private int videoWidth;
    private int videoHeight;
    private int frameRate = 24;
    private int frameDepth = Frame.DEPTH_UBYTE;
    private int frameChannels = 2;

    private Context mContext;

    private View mView;
    private FrameLayout mRecordBtn;

    private SimpleDateFormat mDateFormat = new SimpleDateFormat("yyyy-MM-dd HH-mm-ss");

    private String mInitFilePath;
    private String mInitDirPath;

    public FFmpegVideoRecord(int previewWidth, int previewHeight, Activity activity, View view) {

        mContext = activity.getBaseContext();

        this.previewWidth = previewWidth;
        this.previewHeight = previewHeight;

        videoWidth = 1280;
        videoHeight = 720;

        mCameraId = Camera.CameraInfo.CAMERA_FACING_BACK;
        // At most buffer 10 Frame
        mFrameToRecordQueue = new LinkedBlockingQueue<>(10);
        // At most recycle 2 Frame
        mRecycledFrameQueue = new LinkedBlockingQueue<>(2);
        mRecordFragments = new Stack<>();

        mView = view;
        mRecordBtn = (FrameLayout) mView.findViewById(R.id.button_video);
        mRecordBtn.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startStopRecord();
            }
        });
    }

    public void startStopRecord() {

        if (mRecording) {
            stopRecording();
        } else {
            File sdcardFile = UtilMethods.getSdCardPaths(true, mContext).get(0);
            mInitDirPath = sdcardFile.getAbsolutePath();
            initRecorder();
            startRecorder();
            initAudioRecorder();
            startRecording();
            resumeRecording();
        }
    }

    public void record(byte[] data) {

        if (mRecording) {
            if (audioRecordRunnable == null || !audioRecordRunnable.isRunning()) {
                // wait for AudioRecord to init and start
                mRecordFragments.peek().setStartTimestamp(System.currentTimeMillis());
            } else {
                // pop the current record fragment when calculate total recorded time
                RecordFragment curFragment = mRecordFragments.pop();
                long recordedTime = calculateTotalRecordedTime(mRecordFragments);

                // push it back after calculation
                mRecordFragments.push(curFragment);
                long curRecordedTime = System.currentTimeMillis()
                        - curFragment.getStartTimestamp() + recordedTime;
                long timestamp = 1000 * curRecordedTime;
                Frame frame;
                FrameToRecord frameToRecord = mRecycledFrameQueue.poll();
                if (frameToRecord != null) {
                    frame = frameToRecord.getFrame();
                    frameToRecord.setTimestamp(timestamp);
                } else {
                    frame = new Frame(previewWidth, previewHeight, frameDepth, frameChannels);
                    frameToRecord = new FrameToRecord(timestamp, frame);
                }
                ((ByteBuffer) frame.image[0].position(0)).put(data);

                if (mFrameToRecordQueue.offer(frameToRecord)) {
                    mFrameToRecordCount++;
                }
            }
        }
    }

    public String mDefaultRecordFolder = "_CameraRecord";

    private void initRecorder() {

        File folder = new File(mInitDirPath + File.separator + mDefaultRecordFolder);
        if (!folder.exists()) {
            folder.mkdirs();
        }
        mInitFilePath = mInitDirPath + File.separator + mDefaultRecordFolder
                + File.separator +" Camrecord " + mDateFormat.format(new Date());
        File videoFilePath = new File(mInitFilePath);

        mFrameRecorder = new FFmpegFrameRecorder(videoFilePath, videoWidth, videoHeight, 2);
        mFrameRecorder.setFormat("mp4");
        mFrameRecorder.setSampleRate(sampleAudioRateInHz);
        mFrameRecorder.setFrameRate(frameRate);
        mFrameRecorder.setVideoCodec(avcodec.AV_CODEC_ID_H264);
        mFrameRecorder.setVideoOption("crf", "28");
        mFrameRecorder.setVideoOption("preset", "superfast");
        mFrameRecorder.setVideoOption("tune", "zerolatency");
    }

    private void releaseRecorder() {
        if (mFrameRecorder != null) {
            try {
                mFrameRecorder.release();
            } catch (FFmpegFrameRecorder.Exception e) {
                e.printStackTrace();
            }
            mFrameRecorder = null;
        }
    }

    private void startRecorder() {
        try {
            mFrameRecorder.start();
        } catch (FFmpegFrameRecorder.Exception e) {
            e.printStackTrace();
        }
    }

    private void stopRecorder() {
        if (mFrameRecorder != null) {
            try {
                mFrameRecorder.stop();
            } catch (FFmpegFrameRecorder.Exception e) {
                e.printStackTrace();
            }
        }

        mRecordFragments.clear();
    }

    private void initAudioRecorder() {
        audioRecordRunnable = new AudioRecordRunnable();
    }

    private void releaseAudioRecorder() {
        if (audioRecordRunnable != null) {
            audioRecordRunnable.release();
            audioRecordRunnable = null;
        }
    }

    private void startRecording() {
        mVideoRecordThread = new VideoRecordThread();
        mVideoRecordThread.start();
    }

    public void stopRecording() {

        if (!mRecording) {
            return;
        }

        mRecordFragments.peek().setEndTimestamp(System.currentTimeMillis());
        mRecording = false;
        audioRecordRunnable.stop();

        if (mVideoRecordThread != null && mVideoRecordThread.isRunning()) {
            mVideoRecordThread.stopRunning();
            try {
                mVideoRecordThread.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            mVideoRecordThread = null;
        }

        mFrameToRecordQueue.clear();
        mRecycledFrameQueue.clear();
        stopRecorder();
        releaseRecorder();
        releaseAudioRecorder();
    }

    private void resumeRecording() {
        if (!mRecording) {
            RecordFragment recordFragment = new RecordFragment();
            recordFragment.setStartTimestamp(System.currentTimeMillis());
            mRecordFragments.push(recordFragment);
            mRecording = true;
            new Thread(audioRecordRunnable).start();
        }
    }

    private long calculateTotalRecordedTime(Stack<RecordFragment> recordFragments) {
        long recordedTime = 0;
        for (RecordFragment recordFragment : recordFragments) {
            recordedTime += recordFragment.getDuration();
        }
        return recordedTime;
    }

    class AudioRecordRunnable implements Runnable {

        private boolean isRunning;
        private AudioRecord mAudioRecord;
        private ShortBuffer audioData;
        private CountDownLatch latch;

        public AudioRecordRunnable() {
            int bufferSize = AudioRecord.getMinBufferSize(sampleAudioRateInHz,
                    AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT);
            mAudioRecord = new AudioRecord(MediaRecorder.AudioSource.MIC, sampleAudioRateInHz,
                    AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT, bufferSize);

            audioData = ShortBuffer.allocate(bufferSize);
        }

        @Override
        public void run() {
            latch = new CountDownLatch(1);
            android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_URGENT_AUDIO);

            mAudioRecord.startRecording();

            isRunning = true;
            while (isRunning) {
                if (mRecording && mFrameRecorder != null) {
                    int bufferReadResult = mAudioRecord.read(audioData.array(), 0, audioData.capacity());
                    audioData.limit(bufferReadResult);
                    if (bufferReadResult > 0) {
                        try {
                            mFrameRecorder.recordSamples(audioData);
                        } catch (FFmpegFrameRecorder.Exception | NullPointerException e) {
                            e.printStackTrace();
                        }
                    }
                }
            }
            mAudioRecord.stop();
            latch.countDown();
        }

        public boolean isRunning() {
            return isRunning;
        }

        public void stop() {
            this.isRunning = false;
        }

        public void release() {
            if (latch == null) {
                return;
            }
            try {
                latch.await();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            if (mAudioRecord != null) {
                mAudioRecord.release();
                mAudioRecord = null;
            }
        }
    }

    class VideoRecordThread extends Thread {

        private boolean isRunning;

        @Override
        public void run() {

            isRunning = true;
            FrameToRecord recordedFrame = null;

            while (isRunning || !mFrameToRecordQueue.isEmpty()) {

                try {
                    recordedFrame = mFrameToRecordQueue.take();
                } catch (InterruptedException ie) {
                    break;
                }

                if (mFrameRecorder != null && recordedFrame != null)  {
                    long timestamp = recordedFrame.getTimestamp();
                    if (timestamp > mFrameRecorder.getTimestamp()) {
                        mFrameRecorder.setTimestamp(timestamp);
                    }
                    long startTime = System.currentTimeMillis();
                    try {
                        Frame frame = recordedFrame.getFrame();
                        mFrameRecorder.record(frame);
                    } catch (FFmpegFrameRecorder.Exception e) {
                        e.printStackTrace();
                    }
                    long endTime = System.currentTimeMillis();
                    long processTime = endTime - startTime;
                    mTotalProcessFrameTime += processTime;
                    mRecycledFrameQueue.offer(recordedFrame);
                }
            }
        }

        public void stopRunning() {
            this.isRunning = false;
            if (getState() == WAITING) {
                interrupt();
            }
        }

        public boolean isRunning() {
            return isRunning;
        }
    }

    private class FrameToRecord {
        private long timestamp;
        private Frame frame;

        public FrameToRecord(long timestamp, Frame frame) {
            this.timestamp = timestamp;
            this.frame = frame;
        }

        public long getTimestamp() {
            return timestamp;
        }

        public void setTimestamp(long timestamp) {
            this.timestamp = timestamp;
        }

        public Frame getFrame() {
            return frame;
        }

        public void setFrame(Frame frame) {
            this.frame = frame;
        }
    }

    private class RecordFragment {
        private long startTimestamp;
        private long endTimestamp;

        public void setStartTimestamp(long startTimestamp) {
            this.startTimestamp = startTimestamp;
        }

        public long getStartTimestamp() {
            return startTimestamp;
        }

        public void setEndTimestamp(long endTimestamp) {
            this.endTimestamp = endTimestamp;
        }

        public long getEndTimestamp() {
            return endTimestamp;
        }

        public long getDuration() {
            return endTimestamp - startTimestamp;
        }
    }
}
package com.ssafy.mmmr.businformation.service;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.HashMap;
import java.util.Map;

import org.springframework.batch.core.Job;
import org.springframework.batch.core.JobExecution;
import org.springframework.batch.core.JobParameter;
import org.springframework.batch.core.JobParameters;
import org.springframework.batch.core.JobParametersInvalidException;
import org.springframework.batch.core.launch.JobLauncher;
import org.springframework.batch.core.repository.JobExecutionAlreadyRunningException;
import org.springframework.batch.core.repository.JobInstanceAlreadyCompleteException;
import org.springframework.batch.core.repository.JobRestartException;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

import com.ssafy.mmmr.businformation.repository.BusInformationRepository;
import com.ssafy.mmmr.global.error.code.ErrorCode;
import com.ssafy.mmmr.global.error.exception.BusInformationException;

import lombok.RequiredArgsConstructor;

@Service
@RequiredArgsConstructor
public class BusInformationService {

	private final BusInformationRepository busInformationRepository;
	private final JobLauncher jobLauncher;	// Spring Batch 작업을 실행하는 런처
	private final Job busInformationImportJob;

	@Value("${CSV_UPLOAD_DIR}")
	private String uploadDir;

	public String uploadAndProcessFile(MultipartFile file) {
		try {
			// 업로드 하는 디렉토리가 없으면 업로드 디렉토리 생성
			File directory = new File(uploadDir);
			if (!directory.exists()) {
				directory.mkdirs();
			}

			// 파일 저장
			String originalFilename = file.getOriginalFilename();
			String fileExtension = getFileExtension(originalFilename);

			if (!fileExtension.equals("csv")) {
				throw new BusInformationException(ErrorCode.INVALID_EXTENSION_VALUE);
			}

			String filename = System.currentTimeMillis() + "." + fileExtension;
			Path targetLocation = Paths.get(uploadDir).resolve(filename);
			// 업로드된 파일을 지정된 위치에 저장
			Files.copy(file.getInputStream(), targetLocation, StandardCopyOption.REPLACE_EXISTING);

			// 배치 작업 실행
			Map<String, JobParameter<?>> jobParametersMap = new HashMap<>();
			jobParametersMap.put("time", new JobParameter<>(System.currentTimeMillis(), Long.class));
			jobParametersMap.put("filePath", new JobParameter<>(targetLocation.toString(), String.class));

			JobParameters jobParameters = new JobParameters(jobParametersMap);
			JobExecution jobExecution = jobLauncher.run(busInformationImportJob, jobParameters);
			;

			return "파일 업로드 및 DB 저장 JobId: " + jobExecution.getJobId();
		} catch (IOException e) {
			throw new BusInformationException(ErrorCode.FILE_UPLOAD_ERROR);
		} catch (JobParametersInvalidException | JobExecutionAlreadyRunningException
				 | JobRestartException | JobInstanceAlreadyCompleteException e) {
			throw new BusInformationException(ErrorCode.BATCH_PROCESSING_ERROR);
		}


	}

	// csv 파일인지 검증
	public String getFileExtension(String originalFilename) {
		if (originalFilename == null) {
			return "";
		}
		int dotIndex = originalFilename.lastIndexOf(".");
		if (dotIndex == -1) {
			return "";
		}
		return originalFilename.substring(dotIndex + 1);
	}

}

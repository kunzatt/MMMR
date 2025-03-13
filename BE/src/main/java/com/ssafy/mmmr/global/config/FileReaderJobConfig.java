package com.ssafy.mmmr.global.config;

import javax.sql.DataSource;

import org.springframework.batch.core.Job;
import org.springframework.batch.core.Step;
import org.springframework.batch.core.configuration.annotation.StepScope;
import org.springframework.batch.core.job.builder.JobBuilder;
import org.springframework.batch.core.repository.JobRepository;
import org.springframework.batch.core.step.builder.StepBuilder;
import org.springframework.batch.core.launch.support.RunIdIncrementer;
import org.springframework.batch.item.database.BeanPropertyItemSqlParameterSourceProvider;
import org.springframework.batch.item.database.JdbcBatchItemWriter;
import org.springframework.batch.item.database.builder.JdbcBatchItemWriterBuilder;
import org.springframework.batch.item.file.FlatFileItemReader;
import org.springframework.batch.item.file.LineMapper;
import org.springframework.batch.item.file.builder.FlatFileItemReaderBuilder;
import org.springframework.batch.item.file.mapping.BeanWrapperFieldSetMapper;
import org.springframework.batch.item.file.mapping.DefaultLineMapper;
import org.springframework.batch.item.file.transform.DelimitedLineTokenizer;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.core.io.FileSystemResource;
import org.springframework.transaction.PlatformTransactionManager;

import com.ssafy.mmmr.businformation.dto.BusInformationDto;

import lombok.RequiredArgsConstructor;

@Configuration
@RequiredArgsConstructor
public class FileReaderJobConfig {

	private final JobRepository jobRepository;
	private final PlatformTransactionManager transactionManager;
	private final DataSource dataSource;

	// 배치 청크 사이즈 설정
	private static final int CHUNK_SIZE = 1000;

	@Bean
	public Job busInformationImportJob() {
		return new JobBuilder("busInformationImportJob", jobRepository)
			.incrementer(new RunIdIncrementer())
			.start(busInformationImportStep())
			.build();
	}

	@Bean
	public Step busInformationImportStep() {
		return new StepBuilder("busInformationImportStep", jobRepository)
			.<BusInformationDto, BusInformationDto>chunk(CHUNK_SIZE, transactionManager)
			.reader(busInformationItemReader(null)) // 이 빈은 실제로 사용되지 않지만 선언은 유지
			.writer(busInformationItemWriter())
			.build();
	}

	@Bean
	@StepScope
	public FlatFileItemReader<BusInformationDto> busInformationItemReader(
		@Value("#{jobParameters['filePath']}") String filePath) {

		return new FlatFileItemReaderBuilder<BusInformationDto>()
			.name("busInformationItemReader")
			.resource(new FileSystemResource(filePath))
			.linesToSkip(1) // 헤더 스킵
			.lineMapper(lineMapper())
			.build();
	}

	@Bean
	public LineMapper<BusInformationDto> lineMapper() {
		DefaultLineMapper<BusInformationDto> lineMapper = new DefaultLineMapper<>();

		DelimitedLineTokenizer lineTokenizer = new DelimitedLineTokenizer();
		lineTokenizer.setDelimiter(",");
		lineTokenizer.setNames("routeId", "route", "sequence", "stationId", "station");

		BeanWrapperFieldSetMapper<BusInformationDto> fieldSetMapper = new BeanWrapperFieldSetMapper<>();
		fieldSetMapper.setTargetType(BusInformationDto.class);

		lineMapper.setLineTokenizer(lineTokenizer);
		lineMapper.setFieldSetMapper(fieldSetMapper);

		return lineMapper;
	}

	@Bean
	public JdbcBatchItemWriter<BusInformationDto> busInformationItemWriter() {
		return new JdbcBatchItemWriterBuilder<BusInformationDto>()
			.itemSqlParameterSourceProvider(new BeanPropertyItemSqlParameterSourceProvider<>())
			.sql("INSERT INTO businformations (route_id, route, sequence, station_id, station) " +
				"VALUES (:routeId, :route, :sequence, :stationId, :station)")
			.dataSource(dataSource)
			.build();
	}
}
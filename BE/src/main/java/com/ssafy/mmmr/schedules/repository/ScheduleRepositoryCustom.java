package com.ssafy.mmmr.schedules.repository;

import java.util.List;
import java.util.Optional;

import com.ssafy.mmmr.schedules.entity.ScheduleEntity;

public interface ScheduleRepositoryCustom {

	List<ScheduleEntity> findAllNotDeleted();

	Optional<ScheduleEntity> findByIdAndNotDeleted(Long id);

	List<ScheduleEntity> findByProfileIdAndNotDeleted(Long profileId);

	List<ScheduleEntity> searchByProfileIdAndKeyword(Long profileId, String keyword);
}

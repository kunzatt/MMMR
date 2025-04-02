package com.ssafy.mmmr.transportation.repository;

import java.util.List;
import java.util.Optional;

import org.springframework.data.jpa.repository.JpaRepository;

import com.ssafy.mmmr.transportation.entity.BusEntity;

public interface BusRepository extends JpaRepository<BusEntity, Long> {
	Optional<BusEntity> findByIdAndDeletedFalse(Long id);
}
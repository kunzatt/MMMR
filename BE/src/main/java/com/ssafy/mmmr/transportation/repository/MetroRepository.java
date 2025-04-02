package com.ssafy.mmmr.transportation.repository;

import java.util.List;
import java.util.Optional;

import org.springframework.data.jpa.repository.JpaRepository;

import com.ssafy.mmmr.profiles.entity.ProfileEntity;
import com.ssafy.mmmr.transportation.entity.MetroEntity;

public interface MetroRepository extends JpaRepository<MetroEntity, Long> {

	Optional<MetroEntity> findByIdAndDeletedFalse(Long id);

	List<MetroEntity> findByProfileIdAndDeletedFalse(Long profileId);

}